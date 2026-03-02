#include "rl_page_agent.h"
#include <algorithm>
#include <cmath>

namespace dramsim3 {

// Static member definition
constexpr int RLPageAgent::OFFSETS[RLPAGE_NUM_TILINGS][5];

RLPageAgent::RLPageAgent(int num_banks, SimpleStats& stats)
    : num_banks_(num_banks),
      stats_(stats),
      rng_(42) {  // fixed seed for reproducibility
    // Optimistic initialization: fill CMAC tables with positive initial values
    // This encourages exploration of both actions early on (paper recommendation)
    for (int t = 0; t < RLPAGE_NUM_TILINGS; t++) {
        for (int i = 0; i < RLPAGE_TABLE_SIZE; i++) {
            cmac_[t][i] = RLPAGE_INIT_Q_PER_TILING;
        }
    }
    bank_ctx_.resize(num_banks);
}

RLPageState RLPageAgent::MakeState(int rd_q, int wr_q, int bk_q,
                                    int rh, int sr) const {
    RLPageState s;
    // Quantize to specified bit widths
    s.s1 = std::min(rd_q >> 2, 15);   // 4-bit [0,15], divide by 4
    s.s2 = std::min(wr_q >> 2, 15);   // 4-bit [0,15], divide by 4
    s.s3 = std::min(bk_q, 7);          // 3-bit [0,7]
    s.s4 = std::min(rh, 7);            // 3-bit [0,7]
    s.s5 = std::min(sr, 7);            // 3-bit [0,7]
    return s;
}

int RLPageAgent::CMACIndex(int tiling, RLPageState s, int action) const {
    // Combine state features with tiling-specific offsets
    uint32_t raw = (((s.s1 + OFFSETS[tiling][0]) & 0xF) << 13)
                 | (((s.s2 + OFFSETS[tiling][1]) & 0xF) << 9)
                 | (((s.s3 + OFFSETS[tiling][2]) & 0x7) << 6)
                 | (((s.s4 + OFFSETS[tiling][3]) & 0x7) << 3)
                 | (((s.s5 + OFFSETS[tiling][4]) & 0x7));
    // XOR to differentiate actions
    raw ^= (action ? 0xA5A5u : 0x5A5Au);
    // Hash fold to TABLE_SIZE (256)
    return static_cast<int>((raw ^ (raw >> 8)) & 0xFF);
}

int32_t RLPageAgent::GetQ(RLPageState s, int action) const {
    int32_t sum = 0;
    for (int t = 0; t < RLPAGE_NUM_TILINGS; t++) {
        sum += cmac_[t][CMACIndex(t, s, action)];
    }
    return sum;
}

void RLPageAgent::UpdateQ(RLPageState s, int action, int32_t td_error) {
    // Distribute update across all tilings
    int16_t delta = Clamp16(static_cast<int32_t>(
        RLPAGE_ALPHA * td_error / RLPAGE_NUM_TILINGS));
    for (int t = 0; t < RLPAGE_NUM_TILINGS; t++) {
        int idx = CMACIndex(t, s, action);
        cmac_[t][idx] = Clamp16(static_cast<int32_t>(cmac_[t][idx]) + delta);
    }
}

int RLPageAgent::Decide(int bank_id, int row,
                         int rd_q_depth, int wr_q_depth,
                         int bank_q_depth, int row_hit_count,
                         int same_row_pending) {
    stats_.Increment("rlpage_decisions");

    // 1. Build current state
    RLPageState curr_state = MakeState(rd_q_depth, wr_q_depth,
                                        bank_q_depth, row_hit_count,
                                        same_row_pending);

    // 2. Epsilon-greedy action selection
    int action;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    bool exploring = (dist(rng_) < RLPAGE_EPSILON);

    if (exploring) {
        stats_.Increment("rlpage_explorations");
        std::uniform_int_distribution<int> action_dist(0, 1);
        action = action_dist(rng_);
    } else {
        int32_t q_close = GetQ(curr_state, 0);
        int32_t q_open = GetQ(curr_state, 1);
        action = (q_open >= q_close) ? 1 : 0;
    }

    // 3. If there's a previous decision on this bank, compute reward and do SARSA update
    auto& ctx = bank_ctx_[bank_id];
    if (ctx.valid) {
        int reward = 0;
        if (ctx.action == 1) {
            // Previous: KEEP_OPEN
            if (row == ctx.row) {
                // Same row came back -> row hit, good decision
                reward = 1;
            } else {
                // Different row -> row conflict, bad decision
                reward = -1;
            }
        } else {
            // Previous: CLOSE
            if (row == ctx.row) {
                // Same row came back -> closed too early, bad
                reward = -1;
            } else {
                // Different row -> closed correctly, good
                reward = 1;
            }
        }

        // Track reward statistics
        stats_.Increment("rlpage_rewards");
        if (reward > 0) {
            stats_.Increment("rlpage_positive_rewards");
        } else {
            stats_.Increment("rlpage_negative_rewards");
        }

        // SARSA TD error: r + gamma * Q(s', a') - Q(s, a)
        int32_t q_prev = GetQ(ctx.state, ctx.action);
        int32_t q_curr = GetQ(curr_state, action);
        int32_t td_error = static_cast<int32_t>(reward * 1024)  // scale reward to fixed-point
                         + static_cast<int32_t>(RLPAGE_GAMMA * q_curr)
                         - q_prev;

        UpdateQ(ctx.state, ctx.action, td_error);
        stats_.Increment("rlpage_updates");
    }

    // 4. Record current decision for future reward computation
    ctx.valid = true;
    ctx.state = curr_state;
    ctx.action = action;
    ctx.row = row;

    // 5. Track action statistics
    if (action == 0) {
        stats_.Increment("rlpage_close_count");
    } else {
        stats_.Increment("rlpage_keepopen_count");
    }

    return action;
}

void RLPageAgent::OnActivate(int bank_id, int new_row) {
    // Called when ACT is issued. If the previous decision was KEEP_OPEN,
    // an ACT means a different row is being opened (row conflict occurred).
    // This provides the reward signal for KEEP_OPEN decisions that didn't
    // get resolved at the next cluster-end (e.g., bank was idle then reactivated).
    auto& ctx = bank_ctx_[bank_id];
    if (!ctx.valid) return;

    if (ctx.action == 1) {
        // Previous was KEEP_OPEN, and now ACT fires -> row conflict
        // Build a dummy "terminal" state for SARSA update
        // reward = -1 (kept open but different row came)
        if (new_row != ctx.row) {
            int reward = -1;
            stats_.Increment("rlpage_rewards");
            stats_.Increment("rlpage_negative_rewards");

            // Terminal update: no next state (use Q=0 for terminal)
            int32_t q_prev = GetQ(ctx.state, ctx.action);
            int32_t td_error = static_cast<int32_t>(reward * 1024) - q_prev;

            UpdateQ(ctx.state, ctx.action, td_error);
            stats_.Increment("rlpage_updates");

            // Invalidate context since the bank is now being reactivated
            ctx.valid = false;
        }
        // If new_row == ctx.row, this is a re-activation of the same row
        // (shouldn't happen normally for KEEP_OPEN, but just in case)
    }
    // For CLOSE decisions, ACT is expected (we closed, now activating new row).
    // The reward for CLOSE was already given at the next cluster-end Decide().
}

}  // namespace dramsim3
