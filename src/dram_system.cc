#include "dram_system.h"

#include <assert.h>

namespace dramsim3 {

// alternative way is to assign the id in constructor but this is less
// destructive
int BaseDRAMSystem::total_channels_ = 0;

BaseDRAMSystem::BaseDRAMSystem(Config &config, const std::string &output_dir,
                               std::function<void(uint64_t)> read_callback,
                               std::function<void(uint64_t)> write_callback)
    : read_callback_(read_callback),
      write_callback_(write_callback),
      last_req_clk_(0),
      config_(config),
      timing_(config_),
#ifdef THERMAL
      thermal_calc_(config_),
#endif  // THERMAL
      clk_(0) {
    total_channels_ += config_.channels;

#ifdef ADDR_TRACE
    std::string addr_trace_name = config_.output_prefix + "addr.trace";
    address_trace_.open(addr_trace_name);
#endif
}

int BaseDRAMSystem::GetChannel(uint64_t hex_addr) const {
    hex_addr >>= config_.shift_bits;
    return (hex_addr >> config_.ch_pos) & config_.ch_mask;
}

void BaseDRAMSystem::PrintEpochStats() {
    // first epoch, print bracket
    if (clk_ - config_.epoch_period == 0) {
        std::ofstream epoch_out(config_.json_epoch_name, std::ofstream::out);
        epoch_out << "[";
    }
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->PrintEpochStats();
        std::ofstream epoch_out(config_.json_epoch_name, std::ofstream::app);
        epoch_out << "," << std::endl;
    }
#ifdef THERMAL
    thermal_calc_.PrintTransPT(clk_);
#endif  // THERMAL
    return;
}

void BaseDRAMSystem::PrintStats() {
    // Finish epoch output, remove last comma and append ]
    std::ofstream epoch_out(config_.json_epoch_name, std::ios_base::in |
                                                         std::ios_base::out |
                                                         std::ios_base::ate);
    epoch_out.seekp(-2, std::ios_base::cur);
    epoch_out.write("]", 1);
    epoch_out.close();

    std::ofstream json_out(config_.json_stats_name, std::ofstream::out);
    json_out << "{";

    // close it now so that each channel can handle it
    json_out.close();
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->PrintFinalStats();
        if (i != ctrls_.size() - 1) {
            std::ofstream chan_out(config_.json_stats_name, std::ofstream::app);
            chan_out << "," << std::endl;
        }
    }
    json_out.open(config_.json_stats_name, std::ofstream::app);
    json_out << "}";

#ifdef THERMAL
    thermal_calc_.PrintFinalPT(clk_);
#endif  // THERMAL
}

void BaseDRAMSystem::ResetStats() {
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ResetStats();
    }
}

void BaseDRAMSystem::RegisterCallbacks(
    std::function<void(uint64_t)> read_callback,
    std::function<void(uint64_t)> write_callback) {
    // TODO this should be propagated to controllers
    read_callback_ = read_callback;
    write_callback_ = write_callback;
}

void BaseDRAMSystem::RegisterACTCallback(
    std::function<void(uint64_t, 
                        uint64_t, 
                        uint64_t,
                        uint64_t)> act_callback) {
    act_callback_ = act_callback;
}

JedecDRAMSystem::JedecDRAMSystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback) {
    if (config_.IsHMC()) {
        std::cerr << "Initialized a memory system with an HMC config file!"
                  << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }

    ctrls_.reserve(config_.channels);
    for (auto i = 0; i < config_.channels; i++) {
#ifdef THERMAL
        ctrls_.push_back(new Controller(i, config_, timing_, thermal_calc_));
#else
        ctrls_.push_back(new Controller(i, config_, timing_));
#endif  // THERMAL
    }

    // Initialize row history for all banks across all channels
    int total_banks = config_.channels * config_.ranks * config_.banks;
    row_history_.resize(total_banks);
}

JedecDRAMSystem::~JedecDRAMSystem() {
    for (auto it = ctrls_.begin(); it != ctrls_.end(); it++) {
        delete (*it);
    }
}

bool JedecDRAMSystem::WillAcceptTransaction(uint64_t hex_addr,
                                            bool is_write) const {
    int channel = GetChannel(hex_addr);
    return ctrls_[channel]->WillAcceptTransaction(hex_addr, is_write);
}

bool JedecDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
// Record trace - Record address trace for debugging or other purposes
#ifdef ADDR_TRACE
    address_trace_ << std::hex << hex_addr << std::dec << " "
                   << (is_write ? "WRITE " : "READ ") << clk_ << std::endl;
#endif

    int channel = GetChannel(hex_addr);
    bool ok = ctrls_[channel]->WillAcceptTransaction(hex_addr, is_write);

    assert(ok);
    if (ok) {
        // Row hit distance statistics
        Address addr = config_.AddressMapping(hex_addr);
        int bank_idx = GetBankIndex(channel, addr.rank, addr.bankgroup, addr.bank);
        int current_row = addr.row;

        // Check for row hits in history
        auto& history = row_history_[bank_idx];
        for (size_t i = 0; i < history.count; i++) {
            const auto& record = history.records[i];
            if (record.row == current_row) {
                int distance = static_cast<int>(clk_ - record.timestamp);
                row_hit_distance_histogram_[distance]++;
            }
        }

        // Record current access
        RecordRowAccess(bank_idx, current_row, clk_);

        Transaction trans = Transaction(hex_addr, is_write);
        ctrls_[channel]->AddTransaction(trans);
    }
    last_req_clk_ = clk_;
    return ok;
}

void JedecDRAMSystem::ClockTick() {
    for (size_t i = 0; i < ctrls_.size(); i++) {
        // look ahead and return earlier
        while (true) {
            auto pair = ctrls_[i]->ReturnDoneTrans(clk_);
            if (pair.second == 1) {
                write_callback_(pair.first);
            } else if (pair.second == 0) {
                read_callback_(pair.first);
            } else {
                break;
            }
        }
       // while (true) {
       //     auto actAddr = ctrls_[i]->ReturnACT(clk_);
       //     if (actAddr.row != -1) {
       //         act_callback_(actAddr.channel, actAddr.rank,
       //                         actAddr.bank, actAddr.row);
       //     }
       //     else {
       //         break;
       //     }
       // }
    }
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ClockTick();
    }
    clk_++;

    if (clk_ % config_.epoch_period == 0) {
        PrintEpochStats();
    }
    return;
}

void JedecDRAMSystem::PrintStats() {
    // Call base class PrintStats
    BaseDRAMSystem::PrintStats();
    // Append row hit distance statistics
    PrintRowHitDistanceStats();
}

int JedecDRAMSystem::GetBankIndex(int channel, int rank, int bankgroup, int bank) const {
    return channel * (config_.ranks * config_.banks) +
           rank * config_.banks +
           bankgroup * config_.banks_per_group + bank;
}

void JedecDRAMSystem::RecordRowAccess(int bank_idx, int row, uint64_t timestamp) {
    auto& history = row_history_[bank_idx];
    
    // Check if this row already exists in history, if so just update timestamp
    for (size_t i = 0; i < history.count; i++) {
        if (history.records[i].row == row) {
            history.records[i].timestamp = timestamp;
            return;
        }
    }
    
    // Row not found, add new entry
    history.records[history.head] = {row, timestamp};
    history.head = (history.head + 1) % MAX_ROW_HISTORY;
    if (history.count < MAX_ROW_HISTORY) {
        history.count++;
    }
}

void JedecDRAMSystem::PrintRowHitDistanceStats() const {
    std::ofstream out(config_.txt_stats_name, std::ofstream::app);
    out << "\n###########################################\n";
    out << "## Row Hit Distance Distribution\n";
    out << "###########################################\n";

    // Use a capped, non-uniform binning based on timing constraints.
    int max_distance = config_.tREFI;
    if (max_distance <= 0) {
        max_distance = 10000;
    }
    int base_bin = std::max(8, config_.tCCD_S * 2);

    std::vector<std::pair<int, int>> bins;
    int start = 0;
    int width = base_bin;
    while (start < max_distance) {
        int end = start + width - 1;
        if (end >= max_distance) {
            end = max_distance - 1;
        }
        bins.emplace_back(start, end);
        start = end + 1;
        if (start >= max_distance) {
            break;
        }
        width *= 2;
    }

    std::vector<uint64_t> binned_counts(bins.size(), 0);
    uint64_t overflow_count = 0;
    for (const auto& pair : row_hit_distance_histogram_) {
        int distance = pair.first;
        uint64_t count = pair.second;
        if (distance >= max_distance) {
            overflow_count += count;
            continue;
        }
        for (size_t i = 0; i < bins.size(); i++) {
            if (distance >= bins[i].first && distance <= bins[i].second) {
                binned_counts[i] += count;
                break;
            }
        }
    }

    uint64_t total_hits = 0;
    for (size_t i = 0; i < bins.size(); i++) {
        out << "distance[" << bins[i].first << "-" << bins[i].second << "]: "
            << binned_counts[i] << "\n";
        total_hits += binned_counts[i];
    }
    out << "distance[>=" << max_distance << "]: " << overflow_count << "\n";
    total_hits += overflow_count;
    out << "total_row_hits: " << total_hits << "\n";
    out.close();
}

IdealDRAMSystem::IdealDRAMSystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback),
      latency_(config_.ideal_memory_latency) {}

IdealDRAMSystem::~IdealDRAMSystem() {}

bool IdealDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
    auto trans = Transaction(hex_addr, is_write);
    trans.added_cycle = clk_;
    infinite_buffer_q_.push_back(trans);
    return true;
}

void IdealDRAMSystem::ClockTick() {
    for (auto trans_it = infinite_buffer_q_.begin();
         trans_it != infinite_buffer_q_.end();) {
        if (clk_ - trans_it->added_cycle >= static_cast<uint64_t>(latency_)) {
            if (trans_it->is_write) {
                write_callback_(trans_it->addr);
            } else {
                read_callback_(trans_it->addr);
            }
            trans_it = infinite_buffer_q_.erase(trans_it++);
        }
        if (trans_it != infinite_buffer_q_.end()) {
            ++trans_it;
        }
    }

    clk_++;
    return;
}

}  // namespace dramsim3
