#include "dympl_predictor.h"
#include <algorithm>
#include <cmath>

namespace dramsim3 {

DYMPLPredictor::DYMPLPredictor(int num_banks, SimpleStats& stats)
    : num_banks_(num_banks),
      simple_stats_(stats),
      global_counter_(0),
      brt_(num_banks),
      bank_pred_(num_banks),
      predicted_row_(num_banks, -1),
      wt_page_util_(DYMPL_WT_PAGE_UTIL_SIZE, 0),
      wt_page_hot_(DYMPL_WT_PAGE_HOT_SIZE, 0),
      wt_page_rec_(DYMPL_WT_PAGE_REC_SIZE, 0),
      wt_col_stride_(DYMPL_WT_COL_STRIDE_SIZE, 0),
      wt_page_hitcnt_(DYMPL_WT_PAGE_HITCNT_SIZE, 0),
      wt_bank_rec_(DYMPL_WT_BANK_REC_SIZE, 0),
      wt_bank_hitcnt_(DYMPL_WT_BANK_HITCNT_SIZE, 0) {
    // Initialize PRT: DYMPL_PRT_SETS sets, each with DYMPL_PRT_WAYS ways
    prt_.resize(DYMPL_PRT_SETS);
    for (int s = 0; s < DYMPL_PRT_SETS; s++) {
        prt_[s].resize(DYMPL_PRT_WAYS);
    }
}

int DYMPLPredictor::GetPRTSet(int bank_id) const {
    return bank_id % DYMPL_PRT_SETS;
}

PRTEntry* DYMPLPredictor::FindPRTEntry(int bank_id, int row_id) {
    int set_idx = GetPRTSet(bank_id);
    auto& set = prt_[set_idx];
    for (int w = 0; w < DYMPL_PRT_WAYS; w++) {
        if (set[w].valid && set[w].row_id == row_id) {
            return &set[w];
        }
    }
    return nullptr;
}

PRTEntry* DYMPLPredictor::AllocatePRTEntry(int bank_id, int row_id) {
    int set_idx = GetPRTSet(bank_id);
    auto& set = prt_[set_idx];

    // Find invalid (empty) slot first
    for (int w = 0; w < DYMPL_PRT_WAYS; w++) {
        if (!set[w].valid) {
            set[w] = PRTEntry();
            set[w].valid = true;
            set[w].row_id = row_id;
            TouchPRT(&set[w]);
            return &set[w];
        }
    }

    // All valid: evict LRU entry
    int lru_way = 0;
    uint64_t min_counter = set[0].lru_counter;
    for (int w = 1; w < DYMPL_PRT_WAYS; w++) {
        if (set[w].lru_counter < min_counter) {
            min_counter = set[w].lru_counter;
            lru_way = w;
        }
    }
    simple_stats_.Increment("dympl_prt_evictions");
    set[lru_way] = PRTEntry();
    set[lru_way].valid = true;
    set[lru_way].row_id = row_id;
    TouchPRT(&set[lru_way]);
    return &set[lru_way];
}

void DYMPLPredictor::TouchPRT(PRTEntry* entry) {
    entry->lru_counter = ++global_counter_;
}

void DYMPLPredictor::UpdateRecencyInSet(int set_idx, PRTEntry* accessed) {
    auto& set = prt_[set_idx];
    // Set accessed entry to max recency (15), decrement all others
    for (int w = 0; w < DYMPL_PRT_WAYS; w++) {
        if (!set[w].valid) continue;
        if (&set[w] == accessed) {
            set[w].recency = 15;
        } else {
            set[w].recency = SatDecrement(set[w].recency, 0);
        }
    }
}

int DYMPLPredictor::ClampWeight(int w) {
    if (w < -8) return -8;
    if (w > 7) return 7;
    return w;
}

int DYMPLPredictor::SatIncrement(int val, int max_val) {
    return val < max_val ? val + 1 : max_val;
}

int DYMPLPredictor::SatDecrement(int val, int min_val) {
    return val > min_val ? val - 1 : min_val;
}

bool DYMPLPredictor::Predict(int bank_id, int row, int col) {
    simple_stats_.Increment("dympl_predictions");

    PRTEntry* prt = FindPRTEntry(bank_id, row);
    if (prt == nullptr) {
        // PRT miss: default to OPEN, allocate new entry
        simple_stats_.Increment("dympl_prt_misses");
        AllocatePRTEntry(bank_id, row);
        // Store default prediction state (open, no features to train on)
        auto& pred = bank_pred_[bank_id];
        pred.valid = false;  // no meaningful prediction to train on
        predicted_row_[bank_id] = row;
        simple_stats_.Increment("dympl_predict_open");
        return true;  // keep page open
    }

    simple_stats_.Increment("dympl_prt_hits");
    TouchPRT(prt);

    BRTEntry& brt = brt_[bank_id];

    // Compute feature indices
    int f_page_util = prt->utilization;                           // [0,15]
    int f_page_hot = prt->hotness;                                // [0,31]
    int f_page_rec = prt->recency;                                // [0,15]
    int f_col_stride = prt->stride;                               // [0,15]
    int f_page_hitcnt = prt->hit_count + 8;                       // [-8,+7] → [0,15]
    int f_bank_rec = brt.recency;                                 // [0,15]
    int f_bank_hitcnt = brt.hit_count;                            // [0,255]

    // Perceptron sum
    int sum = wt_page_util_[f_page_util]
            + wt_page_hot_[f_page_hot]
            + wt_page_rec_[f_page_rec]
            + wt_col_stride_[f_col_stride]
            + wt_page_hitcnt_[f_page_hitcnt]
            + wt_bank_rec_[f_bank_rec]
            + wt_bank_hitcnt_[f_bank_hitcnt];

    bool predicted_open = (sum >= 0);

    // Store prediction state for deferred training
    auto& pred = bank_pred_[bank_id];
    pred.valid = true;
    pred.predicted_open = predicted_open;
    pred.sum = sum;
    pred.f_page_util = f_page_util;
    pred.f_page_hot = f_page_hot;
    pred.f_page_rec = f_page_rec;
    pred.f_col_stride = f_col_stride;
    pred.f_page_hitcnt = f_page_hitcnt;
    pred.f_bank_rec = f_bank_rec;
    pred.f_bank_hitcnt = f_bank_hitcnt;

    predicted_row_[bank_id] = row;

    if (predicted_open) {
        simple_stats_.Increment("dympl_predict_open");
    } else {
        simple_stats_.Increment("dympl_predict_close");
    }

    return predicted_open;
}

void DYMPLPredictor::UpdateOnCAS(int bank_id, int row, int col, bool is_row_hit) {
    // Count true_open: pending open prediction and row hit confirms it was correct
    if (is_row_hit && bank_pred_[bank_id].valid && bank_pred_[bank_id].predicted_open) {
        simple_stats_.Increment("dympl_true_open");
        // Keep prediction valid for potential later training at ACT
    }

    PRTEntry* prt = FindPRTEntry(bank_id, row);
    if (prt == nullptr) {
        prt = AllocatePRTEntry(bank_id, row);
    }
    TouchPRT(prt);

    int set_idx = GetPRTSet(bank_id);

    // Update page utilization: sat-increment on CAS
    prt->utilization = SatIncrement(prt->utilization, 15);

    // Update page hotness: sat-increment on CAS
    prt->hotness = SatIncrement(prt->hotness, 31);

    // Update page recency: set to 15, decrement others in set
    UpdateRecencyInSet(set_idx, prt);

    // Update column stride: min(abs(curr_col - last_col), 15)
    if (prt->last_col_id >= 0) {
        int stride = std::abs(col - prt->last_col_id);
        prt->stride = std::min(stride, 15);
    } else {
        prt->stride = 0;
    }
    prt->last_col_id = col;

    // Update page hit count: CAS hit → +1, CAS conflict handled in TrainOnACT
    if (is_row_hit) {
        prt->hit_count = SatIncrement(prt->hit_count, 7);
    }

    // Update bank recency: set this bank to 15, decrement others
    for (int b = 0; b < num_banks_; b++) {
        if (b == bank_id) {
            brt_[b].recency = 15;
        } else {
            brt_[b].recency = SatDecrement(brt_[b].recency, 0);
        }
    }

    // Update bank hit count
    if (is_row_hit) {
        brt_[bank_id].hit_count = SatIncrement(brt_[bank_id].hit_count, 255);
    } else {
        brt_[bank_id].hit_count = SatDecrement(brt_[bank_id].hit_count, 0);
    }
}

void DYMPLPredictor::TrainOnACT(int bank_id, int new_row) {
    auto& pred = bank_pred_[bank_id];
    if (!pred.valid) {
        return;
    }

    simple_stats_.Increment("dympl_train_events");

    // Correctness determination (Paper Section 3.3):
    //   predicted_open + ACT fires → row conflict occurred → WRONG (false_open)
    //   predicted_close + ACT(same_row) → same row re-opened → WRONG (false_close)
    //   predicted_close + ACT(diff_row) → different row needed → CORRECT (true_close)
    // Note: if predicted_open and subsequent CAS are hits, no ACT fires and
    // training doesn't happen (correct open predictions are never trained on,
    // which is handled by the paper's "true_open" being a non-training event
    // unless |sum| < theta — but we'd need CAS-time training for that.
    // The paper's ACT-based training only catches wrong OPEN predictions.)
    bool correct;
    if (pred.predicted_open) {
        // ACT fired on this bank → the open page was conflict-precharged → WRONG
        correct = false;
        simple_stats_.Increment("dympl_false_open");
    } else {
        // predicted_close: page was auto-precharged, check if same row returns
        correct = (new_row != predicted_row_[bank_id]);
        if (correct) {
            simple_stats_.Increment("dympl_true_close");
        } else {
            simple_stats_.Increment("dympl_false_close");
        }
    }

    // Training: update weights if wrong OR |sum| < THETA
    bool need_train = !correct || (std::abs(pred.sum) < DYMPL_THETA);
    if (need_train) {
        int delta;
        if (!correct && pred.predicted_open) {
            // Wrong + predicted open → decrement weights (push toward CLOSE)
            delta = -1;
        } else if (!correct && !pred.predicted_open) {
            // Wrong + predicted close → increment weights (push toward OPEN)
            delta = 1;
        } else {
            // Correct but below theta → reinforce
            delta = pred.predicted_open ? 1 : -1;
        }

        wt_page_util_[pred.f_page_util] = ClampWeight(wt_page_util_[pred.f_page_util] + delta);
        wt_page_hot_[pred.f_page_hot] = ClampWeight(wt_page_hot_[pred.f_page_hot] + delta);
        wt_page_rec_[pred.f_page_rec] = ClampWeight(wt_page_rec_[pred.f_page_rec] + delta);
        wt_col_stride_[pred.f_col_stride] = ClampWeight(wt_col_stride_[pred.f_col_stride] + delta);
        wt_page_hitcnt_[pred.f_page_hitcnt] = ClampWeight(wt_page_hitcnt_[pred.f_page_hitcnt] + delta);
        wt_bank_rec_[pred.f_bank_rec] = ClampWeight(wt_bank_rec_[pred.f_bank_rec] + delta);
        wt_bank_hitcnt_[pred.f_bank_hitcnt] = ClampWeight(wt_bank_hitcnt_[pred.f_bank_hitcnt] + delta);
    }

    // Invalidate the prediction state
    pred.valid = false;
}

void DYMPLPredictor::UpdateOnACT(int bank_id, int new_row) {
    PRTEntry* prt = FindPRTEntry(bank_id, new_row);
    if (prt != nullptr) {
        // Reset utilization on ACT (new activation resets spatial locality counter)
        prt->utilization = 0;
    }

    // Update page hit count: ACT conflict → decrement previous row's hit count
    // The previous row's PRT entry should still exist
    // We don't track "previous row" explicitly here; the hit_count decrement
    // for conflicts is handled implicitly: when UpdateOnCAS is called for a
    // non-row-hit CAS, we don't increment hit_count. The ACT itself indicates
    // a conflict for the previously open row.
    // The paper says: "ACT conflict: -1 for page hit count"
    // We need the previous row for this bank. Use predicted_row_ as proxy.
    if (predicted_row_[bank_id] >= 0 && predicted_row_[bank_id] != new_row) {
        PRTEntry* prev = FindPRTEntry(bank_id, predicted_row_[bank_id]);
        if (prev != nullptr) {
            prev->hit_count = SatDecrement(prev->hit_count, -8);
        }
    }

    // Update predicted_row_ to the new row
    predicted_row_[bank_id] = new_row;
}

}  // namespace dramsim3
