#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Samples kept for RPM averaging
#ifndef ENCODER_RPM_SMOOTHING_SIZE
#define ENCODER_RPM_SMOOTHING_SIZE 4
#endif

// Latch positions (compatible with Matthias Hertel’s Arduino Encoder)
typedef enum {
    ENCODER_LATCH_FOUR3,   /* 4 detents, latch on state 3      (default) */
    ENCODER_LATCH_FOUR0,   /* 4 detents, latch on state 0               */
    ENCODER_LATCH_TWO03    /* 2 detents, latch on states 0 & 3          */
} encoder_latch_mode_t;

// Direction states
typedef enum {
    ENCODER_DIR_NONE = 0,
    ENCODER_DIR_CW   = 1,
    ENCODER_DIR_CCW  = -1
} encoder_direction_t;

// Encoder state and timing data
typedef struct {
    int pin_a;
    int pin_b;
    bool pullup;

    // Internal state
    int8_t  old_state;
    int32_t position;            /* raw internal counter */
    int32_t position_ext;        /* latched "user" position */
    int32_t position_ext_prev;   /* last position returned */
    uint32_t pos_time_ms;        /* time of last latch (ms) */
    uint32_t pos_time_prev_ms;   /* previous latch time (ms) */
    uint64_t pos_time_us;
    uint64_t pos_time_prev_us;
    uint32_t rpm_time_diffs_us[ENCODER_RPM_SMOOTHING_SIZE];
    uint8_t rpm_time_index;
    uint8_t rpm_time_count;
    encoder_latch_mode_t mode;
} encoder_t;

// Public API
void  encoder_init(encoder_t *enc,
                   int pin_a,
                   int pin_b,
                   bool pullup,
                   encoder_latch_mode_t mode);

void  encoder_tick(encoder_t *enc);                    /* Call as fast as you like (e.g. from a 1 kHz timer) */
int32_t encoder_get_position(const encoder_t *enc);    /* Latched count                         */
encoder_direction_t encoder_get_direction(encoder_t *enc); /* Consumes delta                    */
uint32_t encoder_get_millis_between_rotations(const encoder_t *enc);
uint32_t encoder_get_rpm(const encoder_t *enc);

#ifdef __cplusplus
}
#endif
