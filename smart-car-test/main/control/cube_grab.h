#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "motion_types.h"

typedef struct {
    bool detected;
    uint32_t sequence;
    int64_t timestamp_us;
    uint16_t area;
    int16_t x10, y10;
    uint8_t width, height;
} cube_observation_t;

typedef enum { CUBE_IDLE, CUBE_WAIT_ARM, CUBE_OBSERVE, CUBE_PULSE,
    CUBE_SETTLE, CUBE_VERIFY, CUBE_WAIT_ACK, CUBE_WAIT_DONE,
    CUBE_DONE, CUBE_FAILED } cube_grab_state_t;
typedef struct {
    cube_grab_state_t state;
    int64_t started_us, deadline_us;
    uint32_t last_sequence;
    unsigned confirmations;
    motor_command_t pulse;
    int8_t remembered_direction[2];
    uint32_t remembered_sequence[2];
    bool searching;
    int64_t search_deadline;
    bool search_turning, search_moved;
    bool send_ping, send_grab, send_stop, send_release, carrying;
} cube_grab_t;
void cube_grab_start(cube_grab_t *g, int64_t now);
void cube_grab_abort(cube_grab_t *g);
motor_command_t cube_grab_step(cube_grab_t *g, const cube_observation_t *c,
    int distance_mm, bool distance_fresh, bool arm_pong, bool arm_ack,
    bool arm_done, bool arm_error, int64_t now);

motor_command_t cube_delivery_step(cube_grab_t *g, const cube_observation_t *yellow,
    int distance_mm, bool distance_fresh, bool arm_ack, bool arm_done,
    bool arm_error, int64_t now);

void cube_remember_target(cube_grab_t *g, const cube_observation_t *c, bool yellow, int64_t now);
