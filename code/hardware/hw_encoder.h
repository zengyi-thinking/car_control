#ifndef HW_ENCODER_H
#define HW_ENCODER_H

#include "zf_common_headfile.h"

void hw_encoder_init(void);
void hw_encoder_read_counts(int16 counts[4]);
void hw_encoder_counts_to_speed_mps(const int16 counts[4], float dt_s, float speed_mps[4]);

#endif
