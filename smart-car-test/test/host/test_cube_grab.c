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
    for(int i=0;i<3;i++) {
        c.sequence++;c.timestamp_us+=600000;
        zero(cube_grab_step(&g,&c,30,true,false,false,false,false,c.timestamp_us));
        assert(g.send_grab==(i==2));
    }
    g.send_grab=false;
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
    puts("cube grab host tests: PASS");return 0;
}
