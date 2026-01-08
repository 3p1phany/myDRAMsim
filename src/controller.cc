#include "controller.h"
#include <iomanip>
#include <iostream>
#include <limits>

namespace dramsim3 {

#ifdef THERMAL
Controller::Controller(int channel, const Config &config, const Timing &timing,
                       ThermalCalculator &thermal_calc)
#else
Controller::Controller(int channel, const Config &config, const Timing &timing)
#endif  // THERMAL
    : channel_id_(channel),
      clk_(0),
      config_(config),
      simple_stats_(config_, channel_id_),
      channel_state_(config, timing),
      cmd_queue_(channel_id_, config, channel_state_, simple_stats_,
                     (config.row_buf_policy == "CLOSE_PAGE" ? RowBufPolicy::CLOSE_PAGE :
                      config.row_buf_policy == "SMART_CLOSE"? RowBufPolicy::SMART_CLOSE:
                      config.row_buf_policy == "DPM"        ? RowBufPolicy::DPM:
                      config.row_buf_policy == "GS"         ? RowBufPolicy::GS:
                      config.row_buf_policy == "ORACLE"     ? RowBufPolicy::ORACLE     : RowBufPolicy::OPEN_PAGE),this),
      refresh_(config, channel_state_),
#ifdef THERMAL
      thermal_calc_(thermal_calc),
#endif  // THERMAL
      is_unified_queue_(config.unified_queue),
      row_buf_policy_(config.row_buf_policy == "CLOSE_PAGE" ? RowBufPolicy::CLOSE_PAGE :
                      config.row_buf_policy == "SMART_CLOSE"? RowBufPolicy::SMART_CLOSE:
                      config.row_buf_policy == "DPM"        ? RowBufPolicy::DPM:
                      config.row_buf_policy == "GS"         ? RowBufPolicy::GS:
                      config.row_buf_policy == "ORACLE"     ? RowBufPolicy::ORACLE     : RowBufPolicy::OPEN_PAGE),
      last_trans_clk_(0),
      write_draining_(0) {


      issuing_refresh_seq_ = false;
      issuing_sref_seq_ = false; 
    if (is_unified_queue_) {
        unified_queue_.reserve(config_.trans_queue_size);
    } else {
        read_queue_.reserve(config_.trans_queue_size);
        write_buffer_.reserve(config_.trans_queue_size);
    }

#ifdef CMD_TRACE
    std::string trace_file_name = config_.output_prefix + "_ch" +
                                  std::to_string(channel_id_) + "cmd.trace";
    std::cout << "Command Trace write to " << trace_file_name << std::endl;
    cmd_trace_.open(trace_file_name, std::ofstream::out);
#endif  // CMD_TRACE

#ifdef TRANS_TRACE
    std::string trans_trace_file_name = config_.output_prefix + "_ch" +
                                  std::to_string(channel_id_) + "trans.trace";
    std::cout << "trans Trace write to " << trans_trace_file_name << std::endl;
    trans_trace_.open(trans_trace_file_name, std::ofstream::out);
#endif  // TRANS_TRACE
}

std::pair<uint64_t, int> Controller::ReturnDoneTrans(uint64_t clk) {
    auto it = return_queue_.begin();
    while (it != return_queue_.end()) {
        if (clk >= it->complete_cycle) {
            if (it->is_write) {
                simple_stats_.Increment("num_writes_done");
            } else {
                simple_stats_.Increment("num_reads_done");
                simple_stats_.AddValue("read_latency", clk_ - it->added_cycle);
            }
            auto pair = std::make_pair(it->addr, it->is_write);
            it = return_queue_.erase(it);
            return pair;
        } else {
            ++it;
        }
    }
    return std::make_pair(-1, -1);
}

Address Controller::ReturnACT(uint64_t clk) {
    if (act_queue_.begin() != act_queue_.end()) {
        auto act = *act_queue_.begin();
        act_queue_.erase(act_queue_.begin());
        return act;
    }
    return Address();
}

void Controller::ClockTick() {
    // update refresh counter
    refresh_.ClockTick();

    bool cmd_issued = false;
    Command cmd;
    if (channel_state_.IsRefreshWaiting()) {
        cmd = cmd_queue_.FinishRefresh();
    }

    // cannot find a refresh related command or there's no refresh
    if (!cmd.IsValid()) {
        cmd = cmd_queue_.GetCommandToIssue();
    }

    if (cmd.IsValid()) {
    // mark refresh context only for refresh-related commands
        if (channel_state_.IsRefreshWaiting()) {
            issuing_refresh_seq_ = true;
        }
        IssueCommand(cmd);
        issuing_refresh_seq_ = false;
        cmd_issued = true;

        if (config_.enable_hbm_dual_cmd) {
            auto second_cmd = cmd_queue_.GetCommandToIssue();
            if (second_cmd.IsValid()) {
                if (second_cmd.IsReadWrite() != cmd.IsReadWrite()) {
                    IssueCommand(second_cmd);
                    simple_stats_.Increment("hbm_dual_cmds");
                }
            }
        }
    }
    else if(row_buf_policy_==RowBufPolicy::GS){ //GS policy timeout handling, can not issue timeout precharge when there is command issued
        //decrease timeout counter for each bank
        for(int i=0;i<cmd_queue_.num_queues_;i++){
            if(cmd_queue_.timeout_ticking[i] && cmd_queue_.timeout_counter[i]>0){
                cmd_queue_.timeout_counter[i]--;
            }
            //timeout reached, issue precharge
            if(cmd_queue_.timeout_ticking[i] && cmd_queue_.timeout_counter[i]==0){
                //try to send precharge command to close the open row
                auto cmd = cmd_queue_.issued_cmd[i];
                cmd.cmd_type=CommandType::PRECHARGE;
                //check sending precharge timing is ok
                auto& bs=channel_state_.bank_states_[cmd.Rank()][cmd.Bankgroup()][cmd.Bank()];
                if(bs.IsRowOpen() && bs.cmd_timing_[static_cast<int>(CommandType::PRECHARGE)]<=clk_){
                    cmd_queue_.timeout_ticking[i]=false;
                    cmd_queue_.timeout_counter[i]=100;
                    IssueCommand(cmd);
                } 
            }
        }
    }


    // power updates pt 1
    for (int i = 0; i < config_.ranks; i++) {
        if (channel_state_.IsRankSelfRefreshing(i)) {
            simple_stats_.IncrementVec("sref_cycles", i);
        } else {
            bool all_idle = channel_state_.IsAllBankIdleInRank(i);
            if (all_idle) {
                simple_stats_.IncrementVec("all_bank_idle_cycles", i);
                channel_state_.rank_idle_cycles[i] += 1;
            } else {
                simple_stats_.IncrementVec("rank_active_cycles", i);
                // reset
                channel_state_.rank_idle_cycles[i] = 0;
            }
        }
    }

    // power updates pt 2: move idle ranks into self-refresh mode to save power
    if (config_.enable_self_refresh && !cmd_issued) {
        for (auto i = 0; i < config_.ranks; i++) {
            if (channel_state_.IsRankSelfRefreshing(i)) {
                // wake up!
                if (!cmd_queue_.rank_q_empty[i]) {
                    auto addr = Address();
                    addr.rank = i;
                    auto cmd = Command(CommandType::SREF_EXIT, addr, -1);
                    cmd = channel_state_.GetReadyCommand(cmd, clk_);
                    if (cmd.IsValid()) {
                        issuing_sref_seq_ = true;
                        IssueCommand(cmd);
                        issuing_sref_seq_ = false;
                        break;
                    }
                }
            } else {
                if (cmd_queue_.rank_q_empty[i] &&
                    channel_state_.rank_idle_cycles[i] >=
                        config_.sref_threshold) {
                    auto addr = Address();
                    addr.rank = i;
                    auto cmd = Command(CommandType::SREF_ENTER, addr, -1);
                    cmd = channel_state_.GetReadyCommand(cmd, clk_);
                    if (cmd.IsValid()) {
                        issuing_sref_seq_ = true;
                        IssueCommand(cmd);
                        issuing_sref_seq_ = false;
                        break;
                    }
                }
            }
        }
    }

    ScheduleTransaction();
    clk_++;
    cmd_queue_.ClockTick();
    simple_stats_.Increment("num_cycles");
    return;
}

bool Controller::WillAcceptTransaction(uint64_t hex_addr, bool is_write) const {
    if (is_unified_queue_) {
        return unified_queue_.size() < unified_queue_.capacity();
    } else if (!is_write) {
        return read_queue_.size() < read_queue_.capacity();
    } else {
        return write_buffer_.size() < write_buffer_.capacity();
    }
}

bool Controller::AddTransaction(Transaction trans) {
    trans.added_cycle = clk_;
    simple_stats_.AddValue("interarrival_latency", clk_ - last_trans_clk_);
    last_trans_clk_ = clk_;

#ifdef TRANS_TRACE
    auto cmd = TransToCommand(trans);
    trans_trace_ << std::left << std::hex << "0x" <<trans.addr 
                 << std::dec << " " << (trans.is_write? "WRITE":"READ") <<" "
                 << clk_ << " " << cmd << std::endl;
#endif  // TRANS_TRACE

    if (trans.is_write) {
        if (pending_wr_q_.count(trans.addr) == 0) {  // can not merge writes
            pending_wr_q_.insert(std::make_pair(trans.addr, trans));
            if (is_unified_queue_) {
                unified_queue_.push_back(trans);
            } else {
                write_buffer_.push_back(trans);
            }
        }
        trans.complete_cycle = clk_ + 1;
        return_queue_.push_back(trans);
        return true;
    } else {  // read
        // if in write buffer, use the write buffer value
        if (pending_wr_q_.count(trans.addr) > 0) {
            trans.complete_cycle = clk_ + 1;
            return_queue_.push_back(trans);
            return true;
        }
        pending_rd_q_.insert(std::make_pair(trans.addr, trans));
        if (pending_rd_q_.count(trans.addr) == 1) {
            if (is_unified_queue_) {
                unified_queue_.push_back(trans);
            } else {
                read_queue_.push_back(trans);
            }
        }
        return true;
    }
}

void Controller::ScheduleTransaction() {
    // determine whether to schedule read or write
    // read/write arbiter,very simple here, we can make it more advanced and complex TODO
    if (write_draining_ == 0 && !is_unified_queue_) {
        // we basically have an upper and lower threshold for write buffer
        if ((write_buffer_.size() >= 7*write_buffer_.capacity()/8) ||
            (write_buffer_.size() > write_buffer_.capacity()/2 && cmd_queue_.QueueEmpty())) {
            write_draining_ = write_buffer_.size();
        }
    }

    std::vector<Transaction> &queue =
        is_unified_queue_ ? unified_queue_
                          : write_draining_ > 0 ? write_buffer_ : read_queue_;
    for (auto it = queue.begin(); it != queue.end(); it++) {
        auto cmd = TransToCommand(*it);
        if (cmd_queue_.WillAcceptCommand(cmd.Rank(), cmd.Bankgroup(),
                                         cmd.Bank())) {
            if (!is_unified_queue_ && cmd.IsWrite()) {
                // Enforce R->W dependency
                if (pending_rd_q_.count(it->addr) > 0) {
                    write_draining_ = 0;
                    break;
                }
                write_draining_ -= 1;
            }
            cmd_queue_.AddCommand(cmd);
            queue.erase(it);
            break;
        }
    }
}

void Controller::IssueCommand(const Command &cmd) {
#ifdef CMD_TRACE
    cmd_trace_ << std::left << std::setw(18) << clk_ << " " << cmd << std::endl;
#endif  // CMD_TRACE
#ifdef THERMAL
    // add channel in, only needed by thermal module
    thermal_calc_.UpdateCMDPower(channel_id_, cmd, clk_);
#endif  // THERMAL

    // --- Classification & invariant checks for PRE/ACT sources ---
    if (cmd.cmd_type == CommandType::PRECHARGE) {
        if (issuing_refresh_seq_) {
            simple_stats_.Increment("num_pre_for_refresh");
        } else if (issuing_sref_seq_) {
            simple_stats_.Increment("num_pre_for_sref");
        } else {
            simple_stats_.Increment("num_pre_for_demand");
        }
    } else if (cmd.cmd_type == CommandType::ACTIVATE) {
        if (issuing_sref_seq_) {
            simple_stats_.Increment("num_act_for_sref");
        } else {
            simple_stats_.Increment("num_act_for_demand");
        }
    }

    // Invariant (Oracle): demand path must not issue ACT/PRE
    if (row_buf_policy_ == RowBufPolicy::ORACLE &&
        (cmd.cmd_type == CommandType::ACTIVATE || cmd.cmd_type == CommandType::PRECHARGE) &&
        !issuing_refresh_seq_ && !issuing_sref_seq_) {
        std::cerr << "[ORACLE] Demand ACT/PRE issued unexpectedly at clk="
                  << clk_ << " : " << cmd << std::endl;
        std::abort();
    }

    // if read/write, update pending queue and return queue
    if (cmd.IsRead()) {
        auto num_reads = pending_rd_q_.count(cmd.hex_addr);
        if (num_reads == 0) {
            std::cerr << cmd.hex_addr << " not in read queue! " << std::endl;
            exit(1);
        }
        // if there are multiple reads pending return them all
        while (num_reads > 0) {
            auto it = pending_rd_q_.find(cmd.hex_addr);
            it->second.complete_cycle = clk_ + config_.read_delay;
            return_queue_.push_back(it->second);
            pending_rd_q_.erase(it);
            num_reads -= 1;
        }
    } else if (cmd.IsWrite()) {
        // there should be only 1 write to the same location at a time
        auto it = pending_wr_q_.find(cmd.hex_addr);
        if (it == pending_wr_q_.end()) {
            std::cerr << cmd.hex_addr << " not in write queue!" << std::endl;
            exit(1);
        }
        auto wr_lat = clk_ - it->second.added_cycle + config_.write_delay;
        simple_stats_.AddValue("write_latency", wr_lat);
        pending_wr_q_.erase(it);
    } else if (cmd.cmd_type == CommandType::ACTIVATE) {
        act_queue_.push_back(Address(channel_id_, cmd.Rank(), 0, 
                                    (8*cmd.Bank() + cmd.Bankgroup()), 
                                    cmd.Row(), 0));
    }
    // must update stats before states (for row hits)
    UpdateCommandStats(cmd);
    channel_state_.UpdateTimingAndStates(cmd, clk_);
}

Command Controller::TransToCommand(const Transaction &trans)const {
    auto addr = config_.AddressMapping(trans.addr);
    CommandType cmd_type;
    if (row_buf_policy_==RowBufPolicy::CLOSE_PAGE) {
        cmd_type = trans.is_write ? CommandType::WRITE_PRECHARGE : CommandType::READ_PRECHARGE;
    } else {
        cmd_type = trans.is_write ? CommandType::WRITE
                                  : CommandType::READ;
    }
    return Command(cmd_type, addr, trans.addr);
}

int Controller::QueueUsage() const { return cmd_queue_.QueueUsage(); }

void Controller::PrintEpochStats() {
    simple_stats_.Increment("epoch_num");
    simple_stats_.PrintEpochStats();
#ifdef THERMAL
    for (int r = 0; r < config_.ranks; r++) {
        double bg_energy = simple_stats_.RankBackgroundEnergy(r);
        thermal_calc_.UpdateBackgroundEnergy(channel_id_, r, bg_energy);
    }
#endif  // THERMAL
    return;
}

void Controller::PrintFinalStats() {
    simple_stats_.PrintFinalStats();

#ifdef THERMAL
    for (int r = 0; r < config_.ranks; r++) {
        double bg_energy = simple_stats_.RankBackgroundEnergy(r);
        thermal_calc_.UpdateBackgroundEnergy(channel_id_, r, bg_energy);
    }
#endif  // THERMAL
    return;
}

void Controller::UpdateCommandStats(const Command &cmd) {
    switch (cmd.cmd_type) {
        case CommandType::READ:
        case CommandType::READ_PRECHARGE:
            simple_stats_.Increment("num_read_cmds");
            if (channel_state_.RowHitCount(cmd.Rank(), cmd.Bankgroup(),
                                           cmd.Bank()) != 0) {
                simple_stats_.Increment("num_read_row_hits");
            }
            break;
        case CommandType::WRITE:
        case CommandType::WRITE_PRECHARGE:
            simple_stats_.Increment("num_write_cmds");
            if (channel_state_.RowHitCount(cmd.Rank(), cmd.Bankgroup(),
                                           cmd.Bank()) != 0) {
                simple_stats_.Increment("num_write_row_hits");
            }
            break;
        case CommandType::ACTIVATE:
            simple_stats_.Increment("num_act_cmds");
            break;
        case CommandType::PRECHARGE:
            simple_stats_.Increment("num_pre_cmds");
            break;
        case CommandType::REFRESH:
            simple_stats_.Increment("num_ref_cmds");
            break;
        case CommandType::REFRESH_BANK:
            simple_stats_.Increment("num_refb_cmds");
            break;
        case CommandType::SREF_ENTER:
            simple_stats_.Increment("num_srefe_cmds");
            break;
        case CommandType::SREF_EXIT:
            simple_stats_.Increment("num_srefx_cmds");
            break;
        default:
            AbruptExit(__FILE__, __LINE__);
    }
}

}  // namespace dramsim3
