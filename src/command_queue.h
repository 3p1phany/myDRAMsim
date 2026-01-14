#ifndef __COMMAND_QUEUE_H
#define __COMMAND_QUEUE_H

#include <unordered_set>
#include <vector>
#include <deque>
#include "channel_state.h"
#include "common.h"
#include "configuration.h"
#include "simple_stats.h"
namespace dramsim3 {

// ===== GS Timeout Update Constants =====
static constexpr int GS_TIMEOUT_COUNT = 7;
static constexpr int GS_TIMEOUT_VALUES[GS_TIMEOUT_COUNT] = {50, 100, 150, 200, 300, 400, 800};
static constexpr uint64_t GS_ARBITRATION_PERIOD = 30000;
static constexpr int GS_VARIATION_THRESHOLD = 5;

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

struct RowExclusionDetectState {
    int prev_row = -1;
    bool prev_closed_by_timeout = false;
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
};

}  // namespace dramsim3
#endif
