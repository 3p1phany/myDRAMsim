#ifndef __DYMPL_PREDICTOR_H
#define __DYMPL_PREDICTOR_H

#include <cstdint>
#include <cstdlib>
#include <vector>
#include "simple_stats.h"

namespace dramsim3 {

// PRT: 32-way set-associative, 16 sets = 512 entries per channel
static constexpr int DYMPL_PRT_SETS = 16;
static constexpr int DYMPL_PRT_WAYS = 32;

// Perceptron training threshold
static constexpr int DYMPL_THETA = 12;

// Weight table sizes
static constexpr int DYMPL_WT_PAGE_UTIL_SIZE = 16;   // 4-bit feature [0,15]
static constexpr int DYMPL_WT_PAGE_HOT_SIZE = 32;    // 5-bit feature [0,31]
static constexpr int DYMPL_WT_PAGE_REC_SIZE = 16;    // 4-bit feature [0,15]
static constexpr int DYMPL_WT_COL_STRIDE_SIZE = 16;  // 4-bit feature [0,15]
static constexpr int DYMPL_WT_PAGE_HITCNT_SIZE = 16; // 4-bit signed [-8,+7] → index offset +8
static constexpr int DYMPL_WT_BANK_REC_SIZE = 16;    // 4-bit feature [0,15]
static constexpr int DYMPL_WT_BANK_HITCNT_SIZE = 256; // 8-bit feature [0,255]

struct PRTEntry {
    int row_id;           // tag
    int last_col_id;      // last column accessed
    int utilization;      // 4-bit [0,15] spatial locality
    int hotness;          // 5-bit [0,31] lifetime access frequency
    int recency;          // 4-bit [0,15] temporal locality ranking
    int stride;           // 4-bit [0,15] column stride
    int hit_count;        // 4-bit signed [-8,+7] page-level hit/miss tendency
    uint64_t lru_counter; // for LRU replacement
    bool valid;

    PRTEntry() : row_id(-1), last_col_id(-1), utilization(0), hotness(0),
                 recency(0), stride(0), hit_count(0), lru_counter(0),
                 valid(false) {}
};

struct BRTEntry {
    int recency;    // 4-bit [0,15] bank-level temporal locality
    int hit_count;  // 8-bit [0,255] bank-level hit/miss tendency

    BRTEntry() : recency(0), hit_count(0) {}
};

// Stored feature snapshot for deferred training
struct PredictionState {
    bool valid;
    bool predicted_open;  // true = keep open, false = auto-precharge
    int sum;              // perceptron sum at prediction time
    // Feature indices at prediction time
    int f_page_util;
    int f_page_hot;
    int f_page_rec;
    int f_col_stride;
    int f_page_hitcnt;    // already offset by +8 for indexing
    int f_bank_rec;
    int f_bank_hitcnt;

    PredictionState() : valid(false), predicted_open(false), sum(0),
                        f_page_util(0), f_page_hot(0), f_page_rec(0),
                        f_col_stride(0), f_page_hitcnt(0),
                        f_bank_rec(0), f_bank_hitcnt(0) {}
};

class DYMPLPredictor {
public:
    DYMPLPredictor(int num_banks, SimpleStats& stats);

    // Returns true if page should stay OPEN, false if should auto-precharge (CLOSE)
    bool Predict(int bank_id, int row, int col);

    // Update features when a CAS command is processed
    void UpdateOnCAS(int bank_id, int row, int col, bool is_row_hit);

    // Train weights based on ACT outcome, then update features for new activation
    void TrainOnACT(int bank_id, int new_row);
    void UpdateOnACT(int bank_id, int new_row);

private:
    int num_banks_;
    SimpleStats& simple_stats_;
    uint64_t global_counter_;  // monotonic counter for LRU

    // PRT: indexed by [set][way]
    std::vector<std::vector<PRTEntry>> prt_;

    // BRT: indexed by bank_id
    std::vector<BRTEntry> brt_;

    // Per-bank prediction state for deferred training
    std::vector<PredictionState> bank_pred_;

    // Per-bank last predicted/open row (for training correctness check)
    std::vector<int> predicted_row_;

    // 7 weight tables (4-bit signed weights, clamped to [-8, +7])
    std::vector<int> wt_page_util_;
    std::vector<int> wt_page_hot_;
    std::vector<int> wt_page_rec_;
    std::vector<int> wt_col_stride_;
    std::vector<int> wt_page_hitcnt_;
    std::vector<int> wt_bank_rec_;
    std::vector<int> wt_bank_hitcnt_;

    // PRT helpers
    int GetPRTSet(int bank_id) const;
    PRTEntry* FindPRTEntry(int bank_id, int row_id);
    PRTEntry* AllocatePRTEntry(int bank_id, int row_id);
    void TouchPRT(PRTEntry* entry);
    void UpdateRecencyInSet(int set_idx, PRTEntry* accessed);

    // Weight clamping
    static int ClampWeight(int w);
    // Saturating arithmetic helpers
    static int SatIncrement(int val, int max_val);
    static int SatDecrement(int val, int min_val);
};

}  // namespace dramsim3
#endif
