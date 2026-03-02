#ifndef __RL_PAGE_AGENT_H
#define __RL_PAGE_AGENT_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <random>
#include "simple_stats.h"

namespace dramsim3 {

static constexpr int RLPAGE_NUM_TILINGS = 8;
static constexpr int RLPAGE_TABLE_SIZE = 256;
static constexpr int RLPAGE_NUM_ACTIONS = 2;  // 0=CLOSE, 1=KEEP_OPEN

// Hyperparameters (paper-recommended defaults)
static constexpr double RLPAGE_ALPHA = 0.1;
static constexpr double RLPAGE_GAMMA = 0.95;
static constexpr double RLPAGE_EPSILON = 0.05;

// Optimistic initialization: Q = 1/(1-gamma) ~ 20, scaled to fixed-point
// Each tiling contributes init_val/NUM_TILINGS
static constexpr int16_t RLPAGE_INIT_Q_PER_TILING = 256;  // ~2.5 per tiling, sum ~20 in 8-bit fixed

struct RLPageState {
    int s1;  // Read queue depth      [0,15]  4-bit
    int s2;  // Write queue depth     [0,15]  4-bit
    int s3;  // Bank queue depth      [0,7]   3-bit
    int s4;  // Row hit cluster size  [0,7]   3-bit
    int s5;  // Same-row pending      [0,7]   3-bit
};

// Per-bank context for SARSA update chain
struct RLPageBankCtx {
    bool valid = false;       // whether a previous decision exists
    RLPageState state;        // state at previous decision
    int action;               // 0=CLOSE, 1=KEEP_OPEN
    int row;                  // row at previous decision
};

class RLPageAgent {
public:
    RLPageAgent(int num_banks, SimpleStats& stats);

    // Called at cluster-end (row_hit_count==1):
    //   - Computes reward for previous decision on this bank
    //   - Does SARSA update
    //   - Selects new action via epsilon-greedy
    // Returns 0=CLOSE, 1=KEEP_OPEN
    int Decide(int bank_id, int row,
               int rd_q_depth, int wr_q_depth,
               int bank_q_depth, int row_hit_count, int same_row_pending);

    // Called when ACT is issued on a bank:
    //   - If previous decision was KEEP_OPEN, we can now compute reward
    void OnActivate(int bank_id, int new_row);

private:
    int num_banks_;
    SimpleStats& stats_;
    std::mt19937 rng_;

    // CMAC tables: 8 tilings x 256 entries, 16-bit fixed-point weights
    // Shared across all banks (per channel)
    // Total storage: 8 * 256 * 2B = 4 KB
    int16_t cmac_[RLPAGE_NUM_TILINGS][RLPAGE_TABLE_SIZE];

    // Per-bank context for SARSA chain
    std::vector<RLPageBankCtx> bank_ctx_;

    // CMAC tiling offsets (from paper Figure 5(c))
    static constexpr int OFFSETS[RLPAGE_NUM_TILINGS][5] = {
        {0,  0,  0, 0, 0},
        {3,  7,  2, 5, 1},
        {11, 3,  5, 1, 2},
        {7,  13, 1, 3, 3},
        {5,  9,  4, 6, 2},
        {2,  11, 6, 2, 1},
        {13, 5,  3, 7, 3},
        {9,  1,  7, 4, 2},
    };

    int CMACIndex(int tiling, RLPageState s, int action) const;
    int32_t GetQ(RLPageState s, int action) const;
    void UpdateQ(RLPageState s, int action, int32_t td_error);
    RLPageState MakeState(int rd_q, int wr_q, int bk_q, int rh, int sr) const;

    static int16_t Clamp16(int32_t val) {
        if (val > 32767) return 32767;
        if (val < -32768) return -32768;
        return static_cast<int16_t>(val);
    }
};

}  // namespace dramsim3
#endif
