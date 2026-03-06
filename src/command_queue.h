#ifndef __COMMAND_QUEUE_H
#define __COMMAND_QUEUE_H

#include <memory>
#include <unordered_set>
#include <vector>
#include <deque>
#include "channel_state.h"
#include "common.h"
#include "configuration.h"
#include "dympl_predictor.h"
#include "rl_page_agent.h"
#include "simple_stats.h"
namespace dramsim3 {

// ===== GS Timeout Update Constants =====
static constexpr int GS_TIMEOUT_COUNT = 7;
static constexpr int GS_TIMEOUT_VALUES[GS_TIMEOUT_COUNT] = {50, 100, 150, 200, 300, 400, 800};
static constexpr uint64_t GS_ARBITRATION_PERIOD = 30000;
static constexpr int GS_VARIATION_THRESHOLD = 5;
static constexpr int GS_ALIGNED_VARIATION_THRESHOLD = 3;  // Paper Table 2: 3%
static constexpr int GS_ALIGNED_ARBITRATION_REQUESTS = 30000;  // Paper Table 2: 30000 requests

// ===== FAPS-3D Constants =====
static constexpr int FAPS_EPOCH_ACCESSES = 1000;

// Per bank shadow simulation state for timeout update
struct GSShadowState {
    int curr_timeout_idx = 1;  // Default 100 cycles (index 1)
    int hits[GS_TIMEOUT_COUNT] = {0};
    int conflicts[GS_TIMEOUT_COUNT] = {0};

    enum class NextCASState { NONE, HIT, MISS, CONFLICT };
    NextCASState next_cas_state[GS_TIMEOUT_COUNT] = {NextCASState::NONE};

    uint64_t last_cas_cycle = 0;
    int prev_open_row = -1;
};

// ===== Row Exclusion Constants and Structures =====
static constexpr int ROW_EXCLUSION_CAPACITY = 64;

struct RowExclusionEntry {
    int rank;
    int bankgroup;
    int bank;
    int row;
    bool caused_conflict = false;

    bool operator==(const RowExclusionEntry& other) const {
        return rank == other.rank && bankgroup == other.bankgroup &&
               bank == other.bank && row == other.row;
    }
};

struct FAPSBankState {
    int last_accessed_row = -1;     // Hit register: last accessed row
    int potential_hit_count = 0;    // Close-page bank potential hit count
};

struct RowExclusionDetectState {
    int prev_row = -1;
    bool prev_closed_by_timeout = false;
    // === Accuracy tracking fields ===
    int timeout_closed_row = -1;          // Row closed by timeout, awaiting verification
    bool pending_timeout_check = false;   // Flag: waiting for next ACT to verify timeout decision
    bool pending_re_hit_check = false;    // Flag: waiting for next CAS/ACT to verify RE hit
    int re_hit_row = -1;                  // Row protected by RE hit, awaiting verification
};

// ===== ABP (Access Based Predictor) Constants =====
static constexpr int ABP_SETS_PER_BANK = 64;
static constexpr int ABP_WAYS = 4;
static constexpr int ABP_DEFAULT_PRED = 1;   // conservative default: close after 1 access
static constexpr int ABP_MAX_PRED = 255;     // 8-bit saturating counter

struct ABPEntry {
    int row = -1;
    int predicted_count = ABP_DEFAULT_PRED;
    bool valid = false;
    uint64_t lru_counter = 0;
};

struct ABPBankState {
    int current_row_accesses = 0;    // actual accesses to current open row
    int predicted_accesses = ABP_DEFAULT_PRED;  // predicted count from table
    bool predictor_hit = false;      // whether predictor had an entry for this row
    int current_row = -1;            // currently tracked row
};

// ===== CRAFT (Cost-Aware Feedback-driven Adaptive Timeout) Constants =====
static constexpr int CRAFT_T_MIN = 50;
static constexpr int CRAFT_T_MAX = 3200;
static constexpr int CRAFT_BASE_STEP = 50;
static constexpr int CRAFT_INIT_TIMEOUT = 200;
static constexpr int CRAFT_REOPEN_STREAK_MAX = 7;
static constexpr int CRAFT_SHIFT_CAP = 5;

// ===== CRAFT Enhancement Feature Flags (compile-time toggles for ablation) =====
static constexpr bool CRAFT_PHASE_RESET_ENABLED = true;
static constexpr int  CRAFT_PHASE_THRESHOLD = 4;        // consecutive conflicts to trigger fast reset

static constexpr bool CRAFT_QDSD_ENABLED = true;
static constexpr int  CRAFT_QDSD_SCALE_CAP = 4;         // max queue-depth scale factor

static constexpr bool CRAFT_RIGHT_STREAK_ENABLED = true;
static constexpr int  CRAFT_RIGHT_THRESHOLD = 4;         // consecutive right precharges for gentle de-escalation

static constexpr bool CRAFT_RW_ENABLED = true;            // read/write cost differentiation

static constexpr bool CRAFT_STREAK_DECAY_ENABLED = true;  // reopen streak decay on right precharge

struct CraftBankState {
    int timeout_value = CRAFT_INIT_TIMEOUT;
    int reopen_streak = 0;
    int prev_row = -1;
    bool prev_closed_by_timeout = false;
    int conflict_streak = 0;   // [PR] consecutive conflict counter [0-7]
    int right_streak = 0;      // [RS] consecutive right-precharge counter [0-7]
};

// ===== Intel Adaptive Page Policy Constants =====
static constexpr int INTAP_MC_BITS = 4;                        // MC width
static constexpr int INTAP_MC_MAX = (1 << INTAP_MC_BITS) - 1;  // 15
static constexpr int INTAP_MC_INIT = INTAP_MC_MAX / 2;         // 7 (midpoint)
static constexpr int INTAP_HIGH_THRESHOLD = 10;                // MC > 10 => open longer
static constexpr int INTAP_LOW_THRESHOLD = 5;                  // MC < 5 => close sooner
static constexpr int INTAP_CHECK_INTERVAL = 128;               // bank accesses per MC check
static constexpr int INTAP_TR_INIT = 200;                      // initial TR (cycles)
static constexpr int INTAP_TR_STEP = 50;                       // TR adjustment step (cycles)
static constexpr int INTAP_TR_MIN = 50;                        // minimum TR
static constexpr int INTAP_TR_MAX = 3200;                      // maximum TR

struct IntelAdaptiveBankState {
    int timeout_register = INTAP_TR_INIT;       // TR: current timeout value (cycles)
    int mistake_counter  = INTAP_MC_INIT;        // MC: 4-bit saturating counter (0-15)
    int access_counter   = 0;                    // accesses since last MC check
    int prev_row         = -1;                   // last row closed by timeout
    bool prev_closed_by_timeout = false;         // whether last precharge was timeout-driven
};

using CMDIterator = std::vector<Command>::iterator;
using CMDQueue = std::vector<Command>;
enum class QueueStructure { PER_RANK, PER_BANK, SIZE };
class Controller;
class CommandQueue {
   public:
    CommandQueue(int channel_id, const Config& config,
                 const ChannelState& channel_state, SimpleStats& simple_stats,RowBufPolicy top_row_buf_policy,Controller* controller);
    Command GetCommandToIssue();
    Command FinishRefresh();
    void ArbitratePagePolicy();
    void ClockTick();
    bool WillAcceptCommand(int rank, int bankgroup, int bank) const;
    bool AddCommand(Command cmd);
    bool QueueEmpty() const;
    int QueueUsage() const;
    std::vector<bool> rank_q_empty;
    std::vector<CMDQueue> victim_cmds_;
    //row hit r/w command count issued in every schedule interval, including those targeting victim commands
    std::vector<int> true_row_hit_count_;
    // hit open row count
    std::vector<int> demand_row_hit_count_;
    //total r/w command count issued in every schedule interval
    std::vector<int> total_command_count_;
    //reserve for dpm 
    std::vector<RowBufPolicy> row_buf_policy_;
    RowBufPolicy top_row_buf_policy_;
    Controller* controller_;
    bool ArbitratePrecharge(const CMDIterator& cmd_it,
                            const CMDQueue& queue) const;
    bool HasRWDependency(const CMDIterator& cmd_it,
                         const CMDQueue& queue) const;
    Command GetFirstReadyInQueue(CMDQueue& queue) ;
    int GetQueueIndex(int rank, int bankgroup, int bank) const;
    CMDQueue& GetQueue(int rank, int bankgroup, int bank);
    CMDQueue& GetNextQueue();
    void GetRefQIndices(const Command& ref);
    void EraseRWCommand(const Command& cmd,bool autoPRE_added);
    Command PrepRefCmd(const CMDIterator& it, const Command& ref) const;

    QueueStructure queue_structure_;
    const Config& config_;
    const ChannelState& channel_state_;
    SimpleStats& simple_stats_;

    std::vector<Command> issued_cmd;
    std::vector<int> timeout_counter;
    std::vector<char> timeout_ticking;
    std::vector<CMDQueue> queues_;
    std::vector<int> bank_sm;

    // Refresh related data structures
    std::unordered_set<int> ref_q_indices_;
    bool is_in_ref_;

    int num_queues_;
    size_t queue_size_;
    int queue_idx_;
    uint64_t clk_;

    // ===== GS Timeout Update Members =====
    std::vector<GSShadowState> gs_shadow_state_;  // per bank
    std::vector<int> gs_aligned_req_count_;  // per bank request counter for GS_ALIGNED

    // GS Timeout Update functions
    void GS_ProcessACT(int queue_idx, int new_row, uint64_t curr_cycle);
    void GS_ProcessCAS(int queue_idx, uint64_t curr_cycle);
    void GS_ArbitrateTimeout();
    void GetBankFromIndex(int queue_idx, int& rank, int& bankgroup, int& bank) const;
    int GetCurrentTimeout(int queue_idx) const;

    // ===== Row Exclusion Members =====
    std::deque<RowExclusionEntry> row_exclusion_store_;  // per channel, shared by all banks
    std::vector<RowExclusionDetectState> re_detect_state_;  // per bank

    // Row Exclusion functions
    // Note: Detection is done in GS_ProcessACT() per paper Section 4.2
    void RE_AddEntry(const RowExclusionEntry& entry);
    bool RE_IsInStore(int rank, int bankgroup, int bank, int row) const;
    void RE_MarkConflict(int rank, int bankgroup, int bank, int row);
    void RE_RemoveEntry(int rank, int bankgroup, int bank, int row);

    // ===== FAPS-3D Members =====
    std::vector<FAPSBankState> faps_bank_state_;  // per bank
    void FAPS_ArbitratePagePolicy();
    void FAPS_TrackAccess(int queue_idx, int row);

    // ===== DYMPL Predictor =====
    std::unique_ptr<DYMPLPredictor> dympl_predictor_;

    // ===== RL_PAGE Agent =====
    std::unique_ptr<RLPageAgent> rl_page_agent_;

    // ===== CRAFT Members =====
    std::vector<CraftBankState> craft_state_;  // per bank
    int craft_conflict_step_;  // computed from tRP and tRCD
    int craft_gentle_step_;    // [RS] half of conflict step for gentle de-escalation

    // CRAFT functions
    void CRAFT_ProcessACT(int queue_idx, int new_row, bool triggered_by_read);

    // ===== Intel Adaptive Members =====
    std::vector<IntelAdaptiveBankState> intap_state_;  // per bank
    void INTAP_ProcessACT(int bank_idx, int new_row);

    // ===== ABP Members =====
    // Per-bank predictor table: abp_table_[bank][set * ABP_WAYS + way]
    std::vector<std::vector<ABPEntry>> abp_table_;  // per bank
    std::vector<ABPBankState> abp_state_;            // per bank runtime state
    uint64_t abp_lru_clock_ = 0;                     // global LRU timestamp

    // ABP functions
    int  ABP_Lookup(int bank_idx, int row);
    void ABP_Update(int bank_idx, int row, int actual_count);
    void ABP_ProcessACT(int queue_idx, int new_row);
};

}  // namespace dramsim3
#endif
