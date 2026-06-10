#ifndef _mecanum_h_
#define _mecanum_h_

#include "zf_common_typedef.h"

#define MECANUM_HALF_WHEELBASE_M     0.110f
#define MECANUM_HALF_TRACK_M         0.095f
#define MECANUM_GEOMETRY_K_M         (MECANUM_HALF_WHEELBASE_M + MECANUM_HALF_TRACK_M)

/*
 * Unified chassis/body convention:
 * - vx > 0 : vehicle moves forward
 * - vy > 0 : vehicle moves right
 * - omega > 0 : vehicle rotates counter-clockwise
 *
 * Unified wheel order:
 * - wheel[0] = LF
 * - wheel[1] = RF
 * - wheel[2] = LB
 * - wheel[3] = RB
 *
 * This matches the user's learning table:
 *   forward: + + + +
 *   right:   - + + -
 *   left:    + - - +
 *   CW:      + - + -
 *   CCW:     - + - +
 */

void mecanum_inverse(float vx, float vy, float omega, float wheel[4]);
void mecanum_forward(const float wheel[4], float *vx, float *vy, float *omega);
void mecanum_desaturate(float wheel[4], float max_abs_speed);
void mecanum_solve(float vx, float vy, float omega, float wheel[4]);

#endif
