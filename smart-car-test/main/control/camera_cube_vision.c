#include "camera_cube_vision.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
typedef struct { int x,y; } point_t;
static int cross(point_t a,point_t b,point_t c)
{ return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x); }
static int compare(const void *a,const void *b)
{ const point_t *p=a,*q=b; return p->x!=q->x ? p->x-q->x : p->y-q->y; }
static void enqueue(camera_cube_workspace_t *w,int p,unsigned *tail)
{ if (!w->seen[p] && !w->mask[p]) { w->seen[p]=1; w->queue[(*tail)++]=p; } }
cube_observation_t camera_cube_detect(camera_cube_workspace_t *w,
    const uint8_t *pixels,uint32_t sequence,int64_t timestamp_us)
{
    cube_observation_t best={.sequence=sequence,.timestamp_us=timestamp_us};
    for(int i=0;i<CUBE_PIXELS;i++) {
        int v=pixels[i],r=(v>>5)*32+16,g=((v>>2)&7)*32+16,b=(v&3)*64+32;
        w->color[i]=g>=48 && g<=176 && g-r>=15 &&
            (g-b>=15 || (g<=112 && g-r>=32 && g-b>=-16));
        w->dark[i]=w->color[i] && g<=144 && r<=80;
    }
    memcpy(w->work,w->color,CUBE_PIXELS);
    memcpy(w->mask,w->color,CUBE_PIXELS);
    for(int y=1;y<119;y++) for(int x=1;x<159;x++) {
        int v=0;
        for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++) v|=w->color[(y+dy)*160+x+dx];
        w->work[y*160+x]=v;
    }
    for(int y=2;y<118;y++) for(int x=2;x<158;x++) {
        int v=1;
        for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++) v&=w->work[(y+dy)*160+x+dx];
        w->mask[y*160+x]=v;
    }
    memset(w->seen,0,CUBE_PIXELS);
    unsigned head=0,tail=0;
    for(int x=0;x<160;x++) { enqueue(w,x,&tail); enqueue(w,119*160+x,&tail); }
    for(int y=0;y<120;y++) { enqueue(w,y*160,&tail); enqueue(w,y*160+159,&tail); }
    while(head<tail) {
        int p=w->queue[head++],x=p%160,y=p/160;
        if(x) enqueue(w,p-1,&tail);
        if(x<159) enqueue(w,p+1,&tail);
        if(y) enqueue(w,p-160,&tail);
        if(y<119) enqueue(w,p+160,&tail);
    }
    for(int i=0;i<CUBE_PIXELS;i++) w->mask[i]|=!w->seen[i];
    memset(w->seen,0,CUBE_PIXELS);
    for(int seed=0;seed<CUBE_PIXELS;seed++) {
        if(!w->mask[seed] || w->seen[seed]) continue;
        head=0;tail=1;w->queue[0]=seed;w->seen[seed]=1;
        int left=159,right=0,top=119,bottom=0,sumx=0,sumy=0,dark=0,color=0;
        int row_left[120],row_right[120];
        for(int y=0;y<120;y++) {row_left[y]=160;row_right[y]=-1;}
        while(head<tail) {
            int p=w->queue[head++],x=p%160,y=p/160;
            if(x<left) left=x;
            if(x>right) right=x;
            if(y<top) top=y;
            if(y>bottom) bottom=y;
            if(x<row_left[y]) row_left[y]=x;
            if(x>row_right[y]) row_right[y]=x;
            sumx+=x;sumy+=y;dark+=w->dark[p];color+=w->color[p];
            int neighbors[4]={x?p-1:-1,x<159?p+1:-1,y?p-160:-1,y<119?p+160:-1};
            for(int k=0;k<4;k++) {int q=neighbors[k];
                if(q>=0 && w->mask[q] && !w->seen[q]) {w->seen[q]=1;w->queue[tail++]=q;}
            }
        }
        if(tail<30 || dark<4 || dark*100<35*(int)tail || left==0 || top==0 || right==159 || bottom>=117) continue;
        point_t points[240],hull[480];int n=0;
        for(int y=top;y<=bottom;y++) if(row_right[y]>=0) {
            points[n++]=(point_t){row_left[y],y};
            if(row_right[y]!=row_left[y]) points[n++]=(point_t){row_right[y],y};
        }
        qsort(points,n,sizeof(point_t),compare);
        int h=0;
        for(int i=0;i<n;i++) {while(h>=2 && cross(hull[h-2],hull[h-1],points[i])<=0) h--;hull[h++]=points[i];}
        int lower=h+1;
        for(int i=n-2;i>=0;i--) {while(h>=lower && cross(hull[h-2],hull[h-1],points[i])<=0) h--;hull[h++]=points[i];}
        if(h>1) h--;
        double hull_area=0;
        for(int i=0;i<h;i++) hull_area+=hull[i].x*hull[(i+1)%h].y-hull[i].y*hull[(i+1)%h].x;
        hull_area=fabs(hull_area)/2;
        int bw=right-left+1,bh=bottom-top+1;
        double fill=(double)tail/(bw*bh),best_fill=fill,ratio=(double)bw/bh;
        for(int angle=0;angle<90;angle+=5) {
            double a=angle*3.141592653589793/180,c=cos(a),s=sin(a);
            double minx=1e6,maxx=-1e6,miny=1e6,maxy=-1e6;
            for(int i=0;i<h;i++) {
                double x=hull[i].x*c+hull[i].y*s,y=-hull[i].x*s+hull[i].y*c;
                if(x<minx) minx=x;
                if(x>maxx) maxx=x;
                if(y<miny) miny=y;
                if(y>maxy) maxy=y;
            }
            double f=tail/((maxx-minx+1)*(maxy-miny+1));
            if(f>best_fill) {best_fill=f;ratio=(maxx-minx+1)/(maxy-miny+1);}
        }
        while(h>3) {
            double nearest=1e6;int at=0;
            for(int i=0;i<h;i++) {
                point_t a=hull[(i+h-1)%h],b=hull[i],c=hull[(i+1)%h];
                double length=hypot(c.x-a.x,c.y-a.y);
                double distance=fabs(cross(a,c,b))/(length>1?length:1);
                if(distance<nearest) {nearest=distance;at=i;}
            }
            if(nearest>0.06*(bw>bh?bw:bh)) break;
            memmove(hull+at,hull+at+1,(h-at-1)*sizeof(point_t));h--;
        }
        bool angular=h>=4 && h<=7 && tail>=0.8*hull_area && fill>=0.55;
        // The calibrated dark color repair makes the saved approach/grab
        // poses pass the geometric branch; PC-only angle templates remain
        // diagnostics, not an automatic grasp authorization.
        if(ratio<0.5 || ratio>2 || color*10<6*(int)tail || !(best_fill>=0.83 || angular)) continue;
        if(tail>best.area) best=(cube_observation_t){true,sequence,timestamp_us,tail,
            (sumx*10+(int)tail/2)/(int)tail,(sumy*10+(int)tail/2)/(int)tail,bw,bh};
    }
    return best;
}

// Pale yellow calibrated from the user's distant and release-position frames.
cube_observation_t camera_yellow_detect(camera_cube_workspace_t *w,
    const uint8_t *pixels,uint32_t sequence,int64_t timestamp_us)
{
    cube_observation_t best={.sequence=sequence,.timestamp_us=timestamp_us};
    for(int i=0;i<CUBE_PIXELS;i++) {
        int v=pixels[i],r=(v>>5)*32+16,g=((v>>2)&7)*32+16,b=(v&3)*64+32;
        w->mask[i]=r>=176 && g>=176 && b>=96 && r-g>=-32 && r-g<=64 && g-b>=32;
    }
    memset(w->seen,0,CUBE_PIXELS);
    for(int seed=0;seed<CUBE_PIXELS;seed++) {
        if(!w->mask[seed] || w->seen[seed]) continue;
        unsigned head=0,tail=1;w->queue[0]=seed;w->seen[seed]=1;
        int sx=0,sy=0,l=159,r=0,t=119,b=0;
        while(head<tail) {
            int i=w->queue[head++],x=i%160,y=i/160;
            sx+=x;sy+=y;
            if(x<l) l=x;
            if(x>r) r=x;
            if(y<t) t=y;
            if(y>b) b=y;
            int neighbors[4]={x?i-1:-1,x<159?i+1:-1,y?i-160:-1,y<119?i+160:-1};
            for(int k=0;k<4;k++) {
                int n=neighbors[k];
                if(n>=0 && w->mask[n] && !w->seen[n]) {w->seen[n]=1;w->queue[tail++]=n;}
            }
        }
        if(tail<20 || l==0 || r==159 || t==0 || b>=117) continue;
        if(tail>best.area) best=(cube_observation_t){true,sequence,timestamp_us,tail,
            (sx*10+(int)tail/2)/(int)tail,(sy*10+(int)tail/2)/(int)tail,r-l+1,b-t+1};
    }
    return best;
}
