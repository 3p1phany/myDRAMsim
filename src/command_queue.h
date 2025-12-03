#ifndef __COMMAND_QUEUE_H
#define __COMMAND_QUEUE_H

#include <unordered_set>
#include <vector>
#include "channel_state.h"
#include "common.h"
#include "configuration.h"
#include "simple_stats.h"
namespace dramsim3 {

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
    void ClockTick() { clk_ += 1;ArbitratePagePolicy();};
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
   private:
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

    std::vector<CMDQueue> queues_;
    std::vector<int> bank_sm;

    // Refresh related data structures
    std::unordered_set<int> ref_q_indices_;
    bool is_in_ref_;

    int num_queues_;
    size_t queue_size_;
    int queue_idx_;
    uint64_t clk_;


};

}  // namespace dramsim3
#endif
