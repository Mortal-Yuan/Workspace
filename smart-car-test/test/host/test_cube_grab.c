#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "camera_cube_vision.h"
static camera_cube_workspace_t work;
static uint8_t pixels[CUBE_PIXELS];
static void zero(motor_command_t m) { assert(!m.a && !m.b && !m.c); }
int main(int argc,char **argv)
{
    assert(argc==2);
    const char *files[]={"cube-angle-original.rgb332","cube-angle-side.rgb332",
        "cube-angle-side-dark.rgb332","cube-far-dark.rgb332","cube-grab-position.rgb332"};
    for(unsigned i=0;i<5;i++) {
        char path[1024];snprintf(path,sizeof(path),"%s/%s",argv[1],files[i]);
        FILE *f=fopen(path,"rb");assert(f);assert(fread(pixels,1,CUBE_PIXELS,f)==CUBE_PIXELS);fclose(f);
        cube_observation_t c=camera_cube_detect(&work,pixels,1,0);
        printf("%s detected=%d center=%d,%d area=%u\n",files[i],c.detected,c.x10,c.y10,c.area);
        assert(c.detected && c.x10>650 && c.x10<850);
    }
    cube_grab_t g;
    cube_observation_t c={true,1,100000,1557,738,680,44,46};
    cube_grab_start(&g,0);assert(g.send_ping);g.send_ping=false;
    zero(cube_grab_step(&g,&c,30,true,true,false,false,false,100000));
    c.sequence++;c.timestamp_us+=600000;
    zero(cube_grab_step(&g,&c,30,true,false,false,false,false,c.timestamp_us));
    assert(g.send_grab && g.state==CUBE_WAIT_ACK);
    g.send_grab=false;
    zero(cube_grab_step(&g,&c,30,true,false,false,false,false,c.timestamp_us+50));
    assert(!g.send_grab); // Waiting for ACK must not retransmit the grab.
    zero(cube_grab_step(&g,&c,30,true,false,true,false,false,c.timestamp_us+100));
    assert(g.state==CUBE_WAIT_DONE);
    zero(cube_grab_step(&g,&c,30,true,false,false,true,false,c.timestamp_us+200));
    assert(g.state==CUBE_DONE && !g.send_grab);
    cube_grab_start(&g,0);c.sequence=1;c.timestamp_us=100000;c.x10=738;
    zero(cube_grab_step(&g,&c,80,true,true,false,false,false,100000));
    c.sequence++;c.timestamp_us=200000;
    motor_command_t m=cube_grab_step(&g,&c,80,true,false,false,false,false,200000);
    assert(m.a<0 && m.c<0);
    zero(cube_grab_step(&g,&c,19,true,false,false,false,false,210000));
    assert(!g.send_grab);
    c.sequence++;c.timestamp_us=300000;c.x10=1000;
    m=cube_grab_step(&g,&c,80,true,false,false,false,false,300000);
    assert(m.a>0 && m.c<0);
    c.detected=false;
    zero(cube_grab_step(&g,&c,80,true,false,false,false,false,310000));
    cube_grab_start(&g,0);
    zero(cube_grab_step(&g,&c,30,true,false,false,false,false,2000001));
    assert(g.state==CUBE_FAILED);
    cube_grab_start(&g,0);g.state=CUBE_PULSE;
    zero(cube_grab_step(&g,&c,80,true,false,false,false,true,100));
    assert(g.state==CUBE_FAILED && !g.send_grab);
    cube_grab_start(&g,0);g.state=CUBE_WAIT_DONE;
    zero(cube_grab_step(&g,&c,30,true,false,false,false,true,100));
    assert(g.state==CUBE_FAILED && g.send_stop);
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;
    c=(cube_observation_t){true,1,100000,100,738,240,10,12};
    m=cube_grab_step(&g,&c,300,true,false,false,false,false,100000);
    assert(m.a==-550 && m.c==-550 && m.b==0);
    m=cube_grab_step(&g,&c,300,true,false,false,false,false,279999);
    assert(m.a==-550 && m.c==-550);
    zero(cube_grab_step(&g,&c,300,true,false,false,false,false,280000));
    assert(g.state==CUBE_SETTLE);
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;c.sequence=2;c.x10=900;
    m=cube_grab_step(&g,&c,80,true,false,false,false,false,100000);
    assert(m.a==450 && m.c==-450 && g.deadline_us==220000);
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;c.x10=738;
    m=cube_grab_step(&g,&c,80,true,false,false,false,false,100000);
    assert(m.a==-550 && g.deadline_us==220000);
    zero(cube_grab_step(&g,&c,40,true,false,false,false,false,120000));
    assert(g.state==CUBE_SETTLE);
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;
    c=(cube_observation_t){true,1,120000000,100,738,240,10,12};
    m=cube_grab_step(&g,&c,300,true,false,false,false,false,120000000);
    assert(g.state==CUBE_PULSE && m.a==-550 && m.c==-550);
    zero(cube_grab_step(&g,&c,300,false,false,false,false,false,120020000));
    assert(g.state==CUBE_OBSERVE);
    zero(cube_grab_step(&g,&c,300,true,false,false,false,true,120040000));
    assert(g.state==CUBE_FAILED);
    // Near target from the BOOT field run: old vertical gate stalled at y=83.
    const int centers[][2]={{740,830},{618,480},{858,880}};
    for(unsigned n=0;n<3;n++) {
        cube_grab_start(&g,0);g.state=CUBE_OBSERVE;
        c=(cube_observation_t){true,1,100000,1760,centers[n][0],centers[n][1],44,46};
        zero(cube_grab_step(&g,&c,38,true,false,false,false,false,c.timestamp_us));
        assert(g.send_grab && g.state==CUBE_WAIT_ACK);
    }
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;c.x10=859;
    m=cube_grab_step(&g,&c,38,true,false,false,false,false,c.timestamp_us);
    assert(m.a==450 && m.c==-450 && !g.send_grab);
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;c.x10=738;c.y10=881;
    zero(cube_grab_step(&g,&c,38,true,false,false,false,false,c.timestamp_us));
    assert(!g.send_grab && g.confirmations==0);
    c.y10=830;c.sequence++;
    zero(cube_grab_step(&g,&c,19,true,false,false,false,false,c.timestamp_us));
    assert(!g.send_grab);
    zero(cube_grab_step(&g,&c,38,false,false,false,false,false,c.timestamp_us));
    assert(!g.send_grab);
    zero(cube_grab_step(&g,&c,38,true,false,false,false,false,c.timestamp_us+900001));
    assert(!g.send_grab);
    const char *yellow_files[]={"yellow-zone-far.rgb332","yellow-zone-release.rgb332"};
    for(unsigned n=0;n<2;n++) {
        char path[1024];snprintf(path,sizeof(path),"%s/%s",argv[1],yellow_files[n]);
        FILE *f=fopen(path,"rb");assert(f);assert(fread(pixels,1,CUBE_PIXELS,f)==CUBE_PIXELS);fclose(f);
        c=camera_yellow_detect(&work,pixels,1,100000);
        assert(c.detected);
        cube_grab_start(&g,0);g.carrying=true;g.state=CUBE_OBSERVE;
        m=cube_delivery_step(&g,&c,161,true,false,false,false,100000);
        if(n==0) {assert(!g.send_release);assert(m.a || m.c);}
        else {
            zero(m);assert(g.send_release && g.state==CUBE_WAIT_ACK);
            g.send_release=false;
            zero(cube_delivery_step(&g,&c,161,true,true,false,false,110000));
            assert(g.state==CUBE_WAIT_DONE && !g.send_release);
            zero(cube_delivery_step(&g,&c,161,true,false,true,false,120000));
            assert(g.state==CUBE_DONE);
        }
    }
    cube_grab_start(&g,0);g.carrying=true;g.state=CUBE_OBSERVE;
    zero(cube_delivery_step(&g,&c,161,true,false,false,false,1000001));
    assert(!g.send_release);
    zero(cube_delivery_step(&g,&c,161,false,false,false,false,100000));
    assert(!g.send_release);
    g.state=CUBE_WAIT_DONE;
    zero(cube_delivery_step(&g,&c,161,true,false,false,true,100000));
    assert(g.state==CUBE_FAILED && g.send_stop && !g.send_release);
    for(int yellow=0;yellow<2;yellow++) for(int side=-1;side<=1;side+=2) {
        cube_grab_start(&g,0);g.state=CUBE_OBSERVE;g.carrying=yellow;
        c=(cube_observation_t){true,1,100000,100,side<0?300:1200,300,10,10};
        cube_remember_target(&g,&c,yellow,100000);
        c.detected=false;
        if(yellow) m=cube_delivery_step(&g,&c,100,true,false,false,false,100000);
        else m=cube_grab_step(&g,&c,100,true,false,false,false,false,100000);
        zero(m);
        if(yellow) m=cube_delivery_step(&g,&c,100,true,false,false,false,800000);
        else m=cube_grab_step(&g,&c,100,true,false,false,false,false,800000);
        assert(m.a==side*450 && m.c==-side*450 && m.b==0);
        assert(!g.send_grab && !g.send_release);
        c.detected=true;c.sequence++;c.timestamp_us=810000;
        if(yellow) m=cube_delivery_step(&g,&c,100,true,false,false,false,810000);
        else m=cube_grab_step(&g,&c,100,true,false,false,false,false,810000);
        zero(m);assert(!g.searching && g.state==CUBE_SETTLE);
        c.detected=false;
        if(yellow) m=cube_delivery_step(&g,&c,100,false,false,false,false,820000);
        else m=cube_grab_step(&g,&c,100,false,false,false,false,false,820000);
        zero(m);assert(!g.searching);
    }
    cube_grab_start(&g,0);g.state=CUBE_OBSERVE;c.detected=false;
    zero(cube_grab_step(&g,&c,100,true,false,false,false,false,100000));
    zero(cube_grab_step(&g,&c,100,true,false,false,false,false,900000));
    assert(!g.searching);
    // Brief frame gaps before search motion must allow immediate one-frame arrival.
    for(int yellow=0;yellow<2;yellow++) {
        cube_grab_start(&g,0);g.state=CUBE_OBSERVE;g.carrying=yellow;
        c=(cube_observation_t){true,1,100000,100,300,300,10,10};
        cube_remember_target(&g,&c,yellow,100000);c.detected=false;
        if(yellow) zero(cube_delivery_step(&g,&c,100,true,false,false,false,100000));
        else zero(cube_grab_step(&g,&c,100,true,false,false,false,false,100000));
        c=(cube_observation_t){true,2,200000,yellow?410:1700,yellow?840:738,yellow?556:680,40,40};
        if(yellow) zero(cube_delivery_step(&g,&c,100,true,false,false,false,200000));
        else zero(cube_grab_step(&g,&c,35,true,false,false,false,false,200000));
        assert(g.state==CUBE_WAIT_ACK && !g.searching);
        assert(yellow?g.send_release:g.send_grab);
    }
    puts("cube grab host tests: PASS");return 0;
}
