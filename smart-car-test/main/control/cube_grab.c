#include "cube_grab.h"
#include <stdlib.h>
#include <string.h>

void cube_grab_start(cube_grab_t *g, int64_t now)
{
    memset(g,0,sizeof(*g));
    g->state=CUBE_WAIT_ARM;
    g->started_us=now;
    g->deadline_us=now+2000000;
    g->send_ping=true;
}
void cube_grab_abort(cube_grab_t *g)
{
    if (g->state==CUBE_WAIT_ACK || g->state==CUBE_WAIT_DONE) g->send_stop=true;
    g->state=CUBE_FAILED;
    g->send_grab=false;
    g->send_ping=false;
}
motor_command_t cube_grab_step(cube_grab_t *g, const cube_observation_t *c,
    int distance_mm, bool distance_fresh, bool arm_pong, bool arm_ack,
    bool arm_done, bool arm_error, int64_t now)
{
    motor_command_t zero={0};
    if (g->state==CUBE_IDLE || g->state==CUBE_DONE || g->state==CUBE_FAILED) return zero;
    if (arm_error || now-g->started_us>90000000) {
        cube_grab_abort(g); return zero;
    }
    if (g->state==CUBE_WAIT_ARM) {
        if (arm_pong) g->state=CUBE_OBSERVE;
        else if (now>=g->deadline_us) cube_grab_abort(g);
        return zero;
    }
    if (g->state==CUBE_WAIT_ACK || g->state==CUBE_WAIT_DONE) {
        if (arm_done) g->state=CUBE_DONE;
        else if (arm_ack && g->state==CUBE_WAIT_ACK) {
            g->state=CUBE_WAIT_DONE; g->deadline_us=now+20000000;
        } else if (now>=g->deadline_us) cube_grab_abort(g);
        return zero;
    }
    const bool fresh=c && c->detected && now>=c->timestamp_us && now-c->timestamp_us<=900000;
    if (!fresh || !distance_fresh || distance_mm<20) {
        g->state=CUBE_OBSERVE; g->confirmations=0; return zero;
    }
    if (g->state==CUBE_PULSE) {
        if (now<g->deadline_us) {
            if (g->pulse.a<0 && g->pulse.c<0 && distance_mm<=40) {
                g->state=CUBE_SETTLE; g->deadline_us=now+700000; return zero;
            }
            return g->pulse;
        }
        g->state=CUBE_SETTLE; g->deadline_us=now+700000; return zero;
    }
    if (g->state==CUBE_SETTLE) {
        if (now>=g->deadline_us) { g->state=CUBE_OBSERVE; g->last_sequence=c->sequence; }
        return zero;
    }
    if (c->sequence==g->last_sequence) return zero;
    g->last_sequence=c->sequence;
    const int error=c->x10-738;
    const bool centered=abs(error)<=60;
    const bool visual_ready=centered && abs(c->y10-680)<=80 && c->area>=1168 && c->area<=1946;
    if (visual_ready && distance_mm<=40) {
        g->state=CUBE_VERIFY;
        if (++g->confirmations>=3) {
            g->state=CUBE_WAIT_ACK; g->send_grab=true; g->deadline_us=now+2000000;
        }
        return zero;
    }
    g->confirmations=0;
    if (!centered) {
        // Same A/C-opposed yaw vector as the ball alignment policy.
        int direction=error>0 ? 1 : -1;
        g->pulse=(motor_command_t){direction*300,0,-direction*300};
    } else if (distance_mm>40 && c->area<1946) {
        // Existing loaded approach tests needed >=400 to break static friction.
        // Keep a short pulse and re-observe; do not reuse the long ball push.
        g->pulse=(motor_command_t){-400,0,-400};
    } else {
        g->state=CUBE_OBSERVE; return zero;
    }
    g->state=CUBE_PULSE;
    g->deadline_us=now+80000;
    return g->pulse;
}
