#include "command_queue.h"
#include <algorithm>
#include "controller.h"

namespace dramsim3 {

CommandQueue::CommandQueue(int channel_id, const Config& config,
                           const ChannelState& channel_state,
                           SimpleStats& simple_stats,const Controller* controller)
    : rank_q_empty(config.ranks, true),
      controller_(controller),
      config_(config),
      channel_state_(channel_state),
      simple_stats_(simple_stats),
      is_in_ref_(false),
      queue_size_(static_cast<size_t>(config_.cmd_queue_size)),
      queue_idx_(0),
      clk_(0) {
    if (config_.queue_structure == "PER_BANK") {
        queue_structure_ = QueueStructure::PER_BANK;
        num_queues_ = config_.banks * config_.ranks;
    } else if (config_.queue_structure == "PER_RANK") {
        queue_structure_ = QueueStructure::PER_RANK;
        num_queues_ = config_.ranks;
    } else {
        std::cerr << "Unsupportted queueing structure "
                  << config_.queue_structure << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }

    queues_.reserve(num_queues_);
    for (int i = 0; i < num_queues_; i++) {
        auto cmd_queue = std::vector<Command>();
        cmd_queue.reserve(config_.cmd_queue_size);
        queues_.push_back(cmd_queue);
    }
    //do not size victime_cmds for now
    //leave it for furthur investigation
    victim_cmds_.resize(num_queues_);

    total_command_count_.resize(num_queues_);
    true_row_hit_count_.resize(num_queues_);
    demand_row_hit_count_.resize(num_queues_);
    row_buf_policy_.resize(num_queues_);
    //page_policy init for every bank
    for(auto& pp:row_buf_policy_){
        if(controller_->row_buf_policy_==RowBufPolicy::CLOSE_PAGE){
            pp=RowBufPolicy::CLOSE_PAGE;
        }
        else if(controller_->row_buf_policy_==RowBufPolicy::OPEN_PAGE){
            pp=RowBufPolicy::OPEN_PAGE;
        }
        else if(controller_->row_buf_policy_==RowBufPolicy::SMART_CLOSE){
            pp=RowBufPolicy::SMART_CLOSE;
        }
        else if(controller_->row_buf_policy_==RowBufPolicy::DPM){
            pp=RowBufPolicy::OPEN_PAGE;
        }
    }
    //bank_sm
    bank_sm.resize(num_queues_);
    for(auto& sm: bank_sm){
        sm=3;
    }
}

Command CommandQueue::GetCommandToIssue() {
    for (int i = 0; i < num_queues_; i++) {
        auto& queue = GetNextQueue();
        // if we're refreshing, skip the command queues that are involved
        if (is_in_ref_) {
            if (ref_q_indices_.find(queue_idx_) != ref_q_indices_.end()) {
                continue;
            }
        }
        auto cmd = GetFirstReadyInQueue(queue);
        if (cmd.IsValid()) {
            if (cmd.IsReadWrite()) {
                bool autoPRE_added = false;
                //row hit count in command queue
                int row_hit_count=0;
                //check command queue for row hit cmds
                row_hit_count += std::count_if(queue.begin(),queue.end(),[&cmd](Command x){return x.Row() == cmd.Row() && x.IsWrite();});
                //check transaction buffer
                const auto& WB = controller_->write_buffer();
                for(const auto& it:WB){
                    Command cmd_it= controller_ -> TransToCommand(it);
                    if(cmd_it.Channel()   == cmd.Channel() && 
                       cmd_it.Rank()      == cmd.Rank()    && 
                       cmd_it.Bankgroup() == cmd.Bankgroup() && 
                       cmd_it.Bank()      == cmd.Bank()     &&
                       cmd_it.Row()       == cmd.Row() &&
                       queue.size() < queue_size_)
                    row_hit_count ++;
                }

                row_hit_count += std::count_if(queue.begin(),queue.end(),[&cmd](Command x){return x.Row() == cmd.Row() && x.IsRead();});
                const auto& RQ = controller_->read_queue();
                for(const auto& it:RQ){
                    Command cmd_it= controller_ -> TransToCommand(it);
                    if(cmd_it.Channel()   == cmd.Channel() && 
                       cmd_it.Rank()      == cmd.Rank()    && 
                       cmd_it.Bankgroup() == cmd.Bankgroup() && 
                       cmd_it.Bank()      == cmd.Bank()     &&
                       cmd_it.Row()       == cmd.Row()     && 
                       queue.size() < queue_size_)
                    row_hit_count ++;
                }

                //end of row hit command cluster
                //strong indicator of autoPRE!!!
                if(row_buf_policy_[queue_idx_] == RowBufPolicy::SMART_CLOSE){
                    if(row_hit_count==1){
                        cmd.cmd_type = cmd.cmd_type==CommandType::READ ? CommandType::READ_PRECHARGE:
                                       cmd.cmd_type==CommandType::WRITE? CommandType::WRITE_PRECHARGE:cmd.cmd_type;
                        autoPRE_added=true;
                    }
                }

                EraseRWCommand(cmd,autoPRE_added);
                //compute total rw command count for each bank
                total_command_count_[queue_idx_]++;


            }
            return cmd;
        }
    }
    return Command();
}

void CommandQueue::ArbitratePagePolicy(){
    //not in arbitration cycle
    if((clk_%1000 !=0) || clk_ <1000){
        return; 
    }

    if(controller_->row_buf_policy_==RowBufPolicy::DPM){
        std::cout<<"true row hit count:"<<std::endl;
        std::copy(true_row_hit_count_.begin(), true_row_hit_count_.end(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
        std::cout<<"demand row hit count:"<<std::endl;
        std::copy(demand_row_hit_count_.begin(), demand_row_hit_count_.end(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
        std::cout<<"total command count:"<<std::endl;
        std::copy(total_command_count_.begin(), total_command_count_.end(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;

        for(int i=0;i<num_queues_;i++){
            if(row_buf_policy_[i]==RowBufPolicy::OPEN_PAGE){
                // a/b < 0.25
                if(true_row_hit_count_[i]<(total_command_count_[i]>>2)){
                    bank_sm[i]=0;
                    row_buf_policy_[i]=RowBufPolicy::SMART_CLOSE;
                }        
                // a/b < 0.5
                else if(true_row_hit_count_[i]<(total_command_count_[i]>>1)){
                    bank_sm[i] = bank_sm[i]==0 ?  0: bank_sm[i]-1;
                }
                // a/b >= 0.5
                else {
                    bank_sm[i] = bank_sm[i]==3 ?  3: bank_sm[i]+1;
                }

                if(bank_sm[i]<=1){
                    row_buf_policy_[i]=RowBufPolicy::SMART_CLOSE;
                }
                else{
                    row_buf_policy_[i]=RowBufPolicy::OPEN_PAGE;
                }
            }
            else if(row_buf_policy_[i]==RowBufPolicy::SMART_CLOSE){
                // a/b >= 0.75
                if(true_row_hit_count_[i]>=0.75*total_command_count_[i]){
                    bank_sm[i]=3;
                    row_buf_policy_[i]=RowBufPolicy::OPEN_PAGE;
                }
                // a/b < 0.5
                else if(true_row_hit_count_[i]<(total_command_count_[i]>>1)){
                    bank_sm[i] = bank_sm[i]==0 ?  0: bank_sm[i]-1;
                }
                // a/b >= 0.5
                else {
                    bank_sm[i] = bank_sm[i]==3 ?  3: bank_sm[i]+1;
                }

                if(bank_sm[i]>=2){
                    row_buf_policy_[i]=RowBufPolicy::OPEN_PAGE;
                }
                else{
                    row_buf_policy_[i]=RowBufPolicy::SMART_CLOSE;
                }
            }

        }

        std::cout << "row buf policy: ";
        for(int i = 0; i < num_queues_; i++){
            if(row_buf_policy_[i] == RowBufPolicy::OPEN_PAGE){
                std::cout << "O "; 
            }
            else {
                std::cout << "# "; 
            }
        }
        std::cout << std::endl;
    } 

}
Command CommandQueue::FinishRefresh() {
    // we can do something fancy here like clearing the R/Ws
    // that already had ACT on the way but by doing that we
    // significantly pushes back the timing for a refresh
    // so we simply implement an ASAP approach
    auto ref = channel_state_.PendingRefCommand();
    if (!is_in_ref_) {
        GetRefQIndices(ref);
        is_in_ref_ = true;
    }

    // either precharge or refresh
    auto cmd = channel_state_.GetReadyCommand(ref, clk_);

    if (cmd.IsRefresh()) {
        //clear refresh related victims.
        for(auto i:ref_q_indices_){
            victim_cmds_[i].clear();
            total_command_count_[i]=0;
            true_row_hit_count_[i]=0;
            demand_row_hit_count_[i]=0;
        }
        ref_q_indices_.clear();
        is_in_ref_ = false;
    }
    return cmd;
}

bool CommandQueue::ArbitratePrecharge(const CMDIterator& cmd_it,
                                      const CMDQueue& queue) const {
    auto cmd = *cmd_it;

    for (auto prev_itr = queue.begin(); prev_itr != cmd_it; prev_itr++) {
        if (prev_itr->Rank() == cmd.Rank() &&
            prev_itr->Bankgroup() == cmd.Bankgroup() &&
            prev_itr->Bank() == cmd.Bank()) {
            return false;
        }
    }

    bool pending_row_hits_exist = false;
    int open_row =
        channel_state_.OpenRow(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
    for (auto pending_itr = cmd_it; pending_itr != queue.end(); pending_itr++) {
        if (pending_itr->Row() == open_row &&
            pending_itr->Bank() == cmd.Bank() &&
            pending_itr->Bankgroup() == cmd.Bankgroup() &&
            pending_itr->Rank() == cmd.Rank()) {
            pending_row_hits_exist = true;
            break;
        }
    }

    bool rowhit_limit_reached =
        channel_state_.RowHitCount(cmd.Rank(), cmd.Bankgroup(), cmd.Bank()) >=
        4;
    if (!pending_row_hits_exist || rowhit_limit_reached) {
        simple_stats_.Increment("num_ondemand_pres");
        return true;
    }
    return false;
}

bool CommandQueue::WillAcceptCommand(int rank, int bankgroup, int bank) const {
    int q_idx = GetQueueIndex(rank, bankgroup, bank);
    return queues_[q_idx].size() < queue_size_;
}

bool CommandQueue::QueueEmpty() const {
    for (const auto& q : queues_) {
        if (!q.empty()) {
            return false;
        }
    }
    return true;
}


bool CommandQueue::AddCommand(Command cmd) {
    auto& queue = GetQueue(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
    if (queue.size() < queue_size_) {
        queue.push_back(cmd);
        rank_q_empty[cmd.Rank()] = false;
        return true;
    } else {
        int index=GetQueueIndex(cmd.Rank(),cmd.Bankgroup(),cmd.Bank());
        //if cmdq is full, there will not be any chance to schedule incomming row hit requests
        victim_cmds_[index].clear();
        return false;
    }
}

CMDQueue& CommandQueue::GetNextQueue() {
    queue_idx_++;
    if (queue_idx_ == num_queues_) {
        queue_idx_ = 0;
    }
    return queues_[queue_idx_];
}

void CommandQueue::GetRefQIndices(const Command& ref) {
    if (ref.cmd_type == CommandType::REFRESH) {
        if (queue_structure_ == QueueStructure::PER_BANK) {
            for (int i = 0; i < num_queues_; i++) {
                if (i / config_.banks == ref.Rank()) {
                    ref_q_indices_.insert(i);
                }
            }
        } else {
            ref_q_indices_.insert(ref.Rank());
        }
    } else {  // refb
        int idx = GetQueueIndex(ref.Rank(), ref.Bankgroup(), ref.Bank());
        ref_q_indices_.insert(idx);
    }
    return;
}

int CommandQueue::GetQueueIndex(int rank, int bankgroup, int bank) const {
    if (queue_structure_ == QueueStructure::PER_RANK) {
        return rank;
    } else {
        return rank * config_.banks + bankgroup * config_.banks_per_group +
               bank;
    }
}

CMDQueue& CommandQueue::GetQueue(int rank, int bankgroup, int bank) {
    int index = GetQueueIndex(rank, bankgroup, bank);
    return queues_[index];
}

Command CommandQueue::GetFirstReadyInQueue(CMDQueue& queue)  {
    for (auto cmd_it = queue.begin(); cmd_it != queue.end(); cmd_it++) {
        Command cmd = channel_state_.GetReadyCommand(*cmd_it, clk_);
        if (!cmd.IsValid()) {
            continue;
        }

        //compute true row_hit command targeting open row or victim rows
        bool true_row_hit=false;
        // means cmd is a row hit command
        if(cmd.IsReadWrite()){
            // will not happen in normal case
            // if a read does not return, issuing write to the same address is absurd
            if(cmd.IsWrite() && HasRWDependency(cmd_it, queue)){
                continue;
            }
            if (cmd_it->induced_precharge) {
                // row hit already counted when its precharge was scheduled
                cmd_it->induced_precharge = false;
            } else {
                true_row_hit=true;
                demand_row_hit_count_[queue_idx_]++;
            }
        }
        else if (cmd.cmd_type == CommandType::PRECHARGE) {
            if (!ArbitratePrecharge(cmd_it, queue)) {
                continue;
            }
            cmd_it->induced_precharge = true;
            //check victim_cmds to verify whether cmd_it can be a possible row hit  
            for(auto& v:victim_cmds_[queue_idx_]){
                if(v.Row() == cmd_it->Row()){
                    true_row_hit=true;
                }
            }
            //if precharge occurs, it means switching rows
            victim_cmds_[queue_idx_].push_back(cmd);
        } 

        if(true_row_hit){
            true_row_hit_count_[queue_idx_]++;
        }
        return cmd;
    }
    return Command();
}

void CommandQueue::EraseRWCommand(const Command& cmd,bool autoPRE_added) {
    auto& queue = GetQueue(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
    for (auto cmd_it = queue.begin(); cmd_it != queue.end(); cmd_it++) {
        if (cmd.hex_addr == cmd_it->hex_addr) {
            auto no_autoPRE_type_eq = !autoPRE_added && (cmd.cmd_type == cmd_it->cmd_type);
            auto autoPRE_type_eq_rd =  autoPRE_added && (cmd.cmd_type == CommandType::READ_PRECHARGE && cmd_it->cmd_type == CommandType::READ);
            auto autoPRE_type_eq_wr =  autoPRE_added && (cmd.cmd_type == CommandType::WRITE_PRECHARGE && cmd_it->cmd_type == CommandType::WRITE);
            
            if(no_autoPRE_type_eq || autoPRE_type_eq_rd || autoPRE_type_eq_wr){
                queue.erase(cmd_it);
                return;
            }
        }
    }
    std::cerr << "cannot find cmd!" << std::endl;
    exit(1);
}


int CommandQueue::QueueUsage() const {
    int usage = 0;
    for (auto i = queues_.begin(); i != queues_.end(); i++) {
        usage += i->size();
    }
    return usage;
}

bool CommandQueue::HasRWDependency(const CMDIterator& cmd_it,
                                   const CMDQueue& queue) const {
    // Read after write has been checked in controller so we only
    // check write after read here
    for (auto it = queue.begin(); it != cmd_it; it++) {
        if (it->IsRead() && it->Row() == cmd_it->Row() &&
            it->Column() == cmd_it->Column() && it->Bank() == cmd_it->Bank() &&
            it->Bankgroup() == cmd_it->Bankgroup()) {
            return true;
        }
    }
    return false;
}

}  // namespace dramsim3
