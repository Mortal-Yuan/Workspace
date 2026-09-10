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
    g->send_release=false;
    g->send_ping=false;
}
void cube_remember_target(cube_grab_t *g, const cube_observation_t *c, bool yellow, int64_t now)
{
    unsigned i=yellow?1:0;
    if(!c || !c->detected || now<c->timestamp_us || now-c->timestamp_us>900000 ||
       c->sequence==g->remembered_sequence[i]) return;
    g->remembered_sequence[i]=c->sequence;
    if(c->x10<780) g->remembered_direction[i]=-1;
    else if(c->x10>820) g->remembered_direction[i]=1;
}
static motor_command_t search_last_target(cube_grab_t *g,int64_t now)
{
    motor_command_t zero={0};
    int direction=g->remembered_direction[g->carrying?1:0];
    g->state=CUBE_OBSERVE;g->confirmations=0;
    if(!direction) return zero;
    if(!g->searching) {
        g->searching=true;g->search_turning=false;g->search_moved=false;g->search_deadline=now+700000;
    }
    if(now>=g->search_deadline) {
        g->search_turning=!g->search_turning;
        if(g->search_turning) g->search_moved=true;
        g->search_deadline=now+(g->search_turning?120000:700000);
    }
    return g->search_turning?(motor_command_t){direction*450,0,-direction*450}:zero;
}
static void stop_search(cube_grab_t *g,int64_t now)
{
    if(g->searching) {
        g->searching=false;
        // A brief loss before any search motion must not restart settling.
        if(g->search_moved) {
            g->deadline_us=g->search_turning?now+700000:g->search_deadline;
            g->state=now<g->deadline_us?CUBE_SETTLE:CUBE_OBSERVE;
        } else g->state=CUBE_OBSERVE;
        g->search_turning=false;g->search_moved=false;
    }
}
motor_command_t cube_grab_step(cube_grab_t *g, const cube_observation_t *c,
    int distance_mm, bool distance_fresh, bool arm_pong, bool arm_ack,
    bool arm_done, bool arm_error, int64_t now)
{
    motor_command_t zero={0};
    if (g->state==CUBE_IDLE || g->state==CUBE_DONE || g->state==CUBE_FAILED) return zero;
    // Alignment has no total time limit: continue observing and correcting
    // until ready or explicitly stopped. Link/sensor safeguards still apply.
    if (arm_error) {
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
    cube_remember_target(g,c,false,now);
    if (!distance_fresh || distance_mm<20) {
        g->searching=false;g->state=CUBE_OBSERVE;g->confirmations=0;return zero;
    }
    if(!fresh) return search_last_target(g,now);
    stop_search(g,now);
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
    const bool centered=abs(error)<=120;
    const bool visual_ready=centered && abs(c->y10-680)<=200 && c->area>=1168 && c->area<=1946;
    if (visual_ready && distance_mm<=40) {
        // One fresh qualifying observation authorizes one grab request.
        g->confirmations=1;
        g->state=CUBE_WAIT_ACK; g->send_grab=true; g->deadline_us=now+2000000;
        return zero;
    }
    g->confirmations=0;
    if (!centered) {
        // Same A/C-opposed yaw vector as the ball alignment policy.
        int direction=error>0 ? 1 : -1;
        g->pulse=(motor_command_t){direction*450,0,-direction*450};
    } else if (distance_mm>40 && c->area<1946) {
        // Loaded chassis barely moved at 400 in the short-pulse field test.
        g->pulse=(motor_command_t){-550,0,-550};
    } else {
        g->state=CUBE_OBSERVE; return zero;
    }
    g->state=CUBE_PULSE;
    // More launch time at distance; retain short positioning near the block.
    g->deadline_us=now+(distance_mm>100 ? 180000 : 120000);
    return g->pulse;
}

// Delivery uses the user-approved release snapshot, not the cube pickup range.
motor_command_t cube_delivery_step(cube_grab_t *g, const cube_observation_t *c,
    int distance_mm, bool distance_fresh, bool arm_ack, bool arm_done,
    bool arm_error, int64_t now)
{
    motor_command_t zero={0};
    if(g->state==CUBE_DONE || g->state==CUBE_FAILED) return zero;
    if(arm_error) {cube_grab_abort(g);return zero;}
    if(g->state==CUBE_WAIT_ACK || g->state==CUBE_WAIT_DONE) {
        if(arm_done) g->state=CUBE_DONE;
        else if(arm_ack && g->state==CUBE_WAIT_ACK) {
            g->state=CUBE_WAIT_DONE;g->deadline_us=now+5000000;
        } else if(now>=g->deadline_us) cube_grab_abort(g);
        return zero;
    }
    bool fresh=c && c->detected && now>=c->timestamp_us && now-c->timestamp_us<=900000;
    cube_remember_target(g,c,true,now);
    if(!distance_fresh || distance_mm<20) {
        g->searching=false;g->state=CUBE_OBSERVE;return zero;
    }
    if(!fresh) return search_last_target(g,now);
    stop_search(g,now);
    if(g->state==CUBE_PULSE) {
        if(now<g->deadline_us) {
            if(g->pulse.a<0 && g->pulse.c<0 && distance_mm<=40) {
                g->state=CUBE_SETTLE;g->deadline_us=now+700000;return zero;
            }
            return g->pulse;
        }
        g->state=CUBE_SETTLE;g->deadline_us=now+700000;return zero;
    }
    if(g->state==CUBE_SETTLE) {
        if(now>=g->deadline_us) {g->state=CUBE_OBSERVE;g->last_sequence=c->sequence;}
        return zero;
    }
    if(c->sequence==g->last_sequence) return zero;
    g->last_sequence=c->sequence;
    int error=c->x10-840;
    bool centered=abs(error)<=120;
    if(centered && abs(c->y10-556)<=200 && c->area>=307 && c->area<=513) {
        g->state=CUBE_WAIT_ACK;g->send_release=true;g->deadline_us=now+2000000;
        return zero;
    }
    if(!centered) {
        int direction=error>0?1:-1;
        g->pulse=(motor_command_t){direction*450,0,-direction*450};
    } else if(c->area<307 && c->y10<756 && distance_mm>40) {
        g->pulse=(motor_command_t){-550,0,-550};
    } else return zero;
    g->state=CUBE_PULSE;g->deadline_us=now+(distance_mm>100?180000:120000);
    return g->pulse;
}
