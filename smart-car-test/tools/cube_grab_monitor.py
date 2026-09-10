"""Passive 160x120 green block debugging; no motion commands."""
import argparse
import json
import math
from pathlib import Path
import queue
import time
import tkinter as tk
from tkinter import ttk

from usb_camera_monitor import connect_preview, SerialWorker, RGB332_TABLE

RAW_FLAG = 32
CALIBRATED_SHAPE = json.loads(Path(__file__).with_name('cube_shape_calibration.json').read_text(encoding='utf-8'))


def calibrated_shape_match(points, color_mask, width, box):
    l,t,r,b = box
    region = set(points)
    silhouette=[]; green=[]
    for y in range(16):
        for x in range(16):
            sx=l+min(r-l,int((x+.5)*(r-l+1)/16))
            sy=t+min(b-t,int((y+.5)*(b-t+1)/16))
            silhouette.append((sx,sy) in region)
            green.append(bool(color_mask[sy*width+sx]) and (sx,sy) in region)
    def overlap(a,b):
        return sum(x and y for x,y in zip(a,b))/max(1,sum(x or y for x,y in zip(a,b)))
    return overlap(silhouette,CALIBRATED_SHAPE['silhouette']), overlap(green,CALIBRATED_SHAPE['green'])


def repair_mask(mask, width, height):
    """Close one-pixel cracks and fill enclosed dark faces, keeping borders."""
    dilated = bytearray(mask)
    for y in range(1, height-1):
        for x in range(1, width-1):
            i=y*width+x
            dilated[i] = any(mask[i+dy*width+dx] for dy in (-1,0,1) for dx in (-1,0,1))
    closed = bytearray(mask)
    for y in range(2, height-2):
        for x in range(2, width-2):
            i=y*width+x
            closed[i] = all(dilated[i+dy*width+dx] for dy in (-1,0,1) for dx in (-1,0,1))
    exterior = bytearray(len(mask))
    stack = [y*width+x for y in range(height) for x in range(width)
             if (x in (0,width-1) or y in (0,height-1)) and not closed[y*width+x]]
    for i in stack: exterior[i]=1
    while stack:
        i=stack.pop(); x,y=i%width,i//width
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if 0<=nx<width and 0<=ny<height:
                j=ny*width+nx
                if not closed[j] and not exterior[j]:
                    exterior[j]=1; stack.append(j)
    return bytearray(a or not b for a,b in zip(closed,exterior))


def polygon_shape(points, tolerance):
    def cross(a,b,c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    ordered=sorted(set(points))
    lower=[]; upper=[]
    for half, source in ((lower,ordered),(upper,reversed(ordered))):
        for p in source:
            while len(half)>=2 and cross(half[-2],half[-1],p)<=0: half.pop()
            half.append(p)
    hull=lower[:-1]+upper[:-1]
    area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(hull,hull[1:]+hull[:1])))/2
    polygon=hull[:]
    while len(polygon)>3:
        distances=[]
        for i,b in enumerate(polygon):
            a,c=polygon[i-1],polygon[(i+1)%len(polygon)]
            distances.append(abs(cross(a,c,b))/max(1,math.dist(a,c)))
        smallest=min(distances)
        if smallest>tolerance: break
        polygon.pop(distances.index(smallest))
    return area, polygon


def detect(pixels, width=160, height=120, dominance=15, minimum_area=30):
    """RGB332-aware color components, with 2D block-shape screening.

    Quantization intervals are compared at their midpoints instead of expanded
    display colors, whose blue quantization is much coarser than red/green.
    A single camera image cannot establish that a target is a 3D cube.
    """
    if len(pixels) != width * height:
        raise ValueError('Invalid image size')
    lut = []
    dark_lut = []
    for v in range(256):
        r, g, b = (v >> 5) * 32 + 16, ((v >> 2) & 7) * 32 + 16, (v & 3) * 64 + 32
        # Blue has only two bits: a real dark green face can have decoded
        # midpoints G=80,B=96. Allow one half-blue-bin tolerance only when
        # dark and distinctly greener than red; never admit bright gray.
        green = (48 <= g <= 176 and g-r >= dominance and
                 (g-b >= dominance or (g <= 112 and g-r >= max(32,dominance) and g-b >= -16)))
        lut.append(green)
        dark_lut.append(green and g <= 144 and r <= 80)
    color_mask = bytearray(lut[v] for v in pixels)
    mask = repair_mask(color_mask, width, height)
    seen = bytearray(len(mask))
    candidates = []
    for seed, yes in enumerate(mask):
        if not yes or seen[seed]:
            continue
        seen[seed] = 1
        stack = [seed]
        points = []
        while stack:
            p = stack.pop()
            x, y = p % width, p // width
            points.append((x, y))
            for nx, ny in ((x-1,y), (x+1,y), (x,y-1), (x,y+1)):
                if 0 <= nx < width and 0 <= ny < height:
                    q = ny * width + nx
                    if mask[q] and not seen[q]:
                        seen[q] = 1
                        stack.append(q)
        area = len(points)
        if area < minimum_area:
            continue
        dark_pixels = sum(dark_lut[pixels[y*width+x]] for x,y in points)
        dark_support = dark_pixels/area
        if dark_pixels < 4 or dark_support < 0.35:
            continue
        xs, ys = zip(*points)
        left, right, top, bottom = min(xs), max(xs), min(ys), max(ys)
        bw, bh = right-left+1, bottom-top+1
        fill = area / (bw*bh)
        # Find the most compact oriented bounding rectangle. This accepts
        # tilted square faces but rejects circles (rectangle fill ~0.785).
        best_fill = fill
        best_ratio = bw / bh
        boundary = [(x,y) for x,y in points if x == left or x == right or
                    y == top or y == bottom or not all(mask[(y+dy)*width+x+dx]
                    for dx,dy in ((-1,0),(1,0),(0,-1),(0,1)))]
        for angle in range(0, 90, 5):
            c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
            us = [x*c+y*s for x,y in boundary]
            vs = [-x*s+y*c for x,y in boundary]
            w, h = max(us)-min(us)+1, max(vs)-min(vs)+1
            f = area/(w*h)
            if f > best_fill:
                best_fill, best_ratio = f, w/h
        touches = left == 0 or top == 0 or right == width-1 or bottom == height-1
        hull_area, polygon = polygon_shape(boundary,max(bw,bh)*0.06)
        solidity=min(1.0,area/max(1,hull_area))
        # A perspective cube often has a four-to-six-corner silhouette; allow
        # one extra corner for quantization/shadow along an edge and use
        # convex polygon support as well as rectangular faces. Circles retain
        # more corners at this scale and are not accepted by this branch.
        angular = 4 <= len(polygon) <= 7 and solidity >= 0.8 and fill >= 0.55
        color_support = sum(color_mask[y*width+x] for x,y in points)/area
        shape_match, green_match = calibrated_shape_match(points,color_mask,width,[left,top,right,bottom])
        calibrated = (shape_match >= 0.88 and green_match >= 0.65
                      and 0.5 <= color_support <= 0.8 and solidity >= 0.8)
        accepted = 0.5 <= best_ratio <= 2.0 and not touches and (
            ((best_fill >= 0.83 or angular) and color_support >= 0.6) or calibrated)
        candidates.append(dict(box=[left,top,right,bottom], center=[sum(xs)/area,sum(ys)/area],
                               area=area, fill=round(best_fill,3), accepted=accepted,
                               solidity=round(solidity,3), polygon=polygon, color_support=round(color_support,3),
                               calibrated=calibrated, shape_match=round(shape_match,3), green_match=round(green_match,3),
                               dark_support=round(dark_support,3),
                               reason='物块候选' if accepted else ('边缘截断' if touches else '形状待确认')))
    candidates.sort(key=lambda c: (c['accepted'], c['area']), reverse=True)
    return mask, candidates


class Tracker:
    MEMORY_SECONDS = 2.0
    MAX_ASSISTED_FRAMES = 2

    def __init__(self):
        self.previous = None
        self.count = 0
        self.last_time = 0
        self.sequence = None
        self.anchor = None
        self.anchor_time = 0
        self.assisted_frames = 0
        self.target = None
        self.assisted = False

    @staticmethod
    def matches(candidate, reference):
        if not reference:
            return False
        a, b = candidate['box'], reference['box']
        return (math.dist(candidate['center'], reference['center']) < 12
                and 0.6 < candidate['area']/reference['area'] < 1.7
                and all(0.6 < (a[i+2]-a[i]+1)/(b[i+2]-b[i]+1) < 1.7
                        for i in (0, 1)))

    def memory_remaining(self, now):
        return max(0, self.MEMORY_SECONDS-(now-self.anchor_time)) if self.anchor else 0

    def update(self, candidates, sequence, now):
        if sequence == self.sequence:
            return self.count
        self.sequence = sequence
        if not self.memory_remaining(now) or now-self.last_time >= 2:
            self.anchor = None
        reference = self.anchor or self.previous
        nearby = sorted((c for c in candidates if self.matches(c, reference)),
                        key=lambda c: math.dist(c['center'], reference['center']))
        target = next((c for c in nearby if c['accepted']), None)
        self.assisted = False
        if target is None and self.anchor and self.assisted_frames < self.MAX_ASSISTED_FRAMES:
            target = next((c for c in nearby if c['reason'] == '形状待确认'
                           and c['fill'] >= 0.4 and c['solidity'] >= 0.65), None)
            self.assisted = target is not None
        if target is None:
            target = next((c for c in candidates if c['accepted']), None)
        same = target and now-self.last_time < 2 and self.matches(target, self.previous)
        remembered = target and self.matches(target, self.anchor)
        self.count = max(3, self.count+1) if remembered else (self.count+1 if same else (1 if target else 0))
        if target:
            if self.assisted:
                self.assisted_frames += 1
            elif self.count >= 3:
                self.anchor = dict(target)
                self.anchor_time = now
                self.assisted_frames = 0
            else:
                self.anchor = None
        self.target = target
        self.previous, self.last_time = target, now
        return self.count


class Window:
    def __init__(self, root, worker, messages):
        self.root, self.worker, self.messages = root, worker, messages
        root.title('抓取物块识别调试 — 160×120')
        self.status = tk.StringVar(value='等待无标记摄像头图像……')
        ttk.Label(root, text='绿色正方体 · 抓取识别调试', font=('Microsoft YaHei',16)).pack(pady=8)
        ttk.Label(root, text='左：原图与候选框    右：暗绿色分割结果    分析分辨率：160×120').pack()
        row = ttk.Frame(root); row.pack(padx=12,pady=10)
        self.canvas = tk.Canvas(row,width=640,height=480,bg='#202020'); self.canvas.pack(side='left')
        self.mask_canvas = tk.Canvas(row,width=320,height=240,bg='#202020'); self.mask_canvas.pack(side='left',padx=10)
        self.dominance = tk.IntVar(value=15)
        self.area = tk.IntVar(value=30)
        controls = ttk.Frame(root); controls.pack(fill='x',padx=15)
        tk.Scale(controls,label='绿色优势（越大越严格）',from_=0,to=64,orient='horizontal',variable=self.dominance,length=280).pack(side='left')
        tk.Scale(controls,label='最小面积（像素）',from_=10,to=500,orient='horizontal',variable=self.area,length=280).pack(side='left')
        ttk.Button(controls,text='保存当前画面与识别数据',command=self.save).pack(side='left',padx=15)
        actions=ttk.Frame(root); actions.pack(pady=4)
        ttk.Button(actions,text='启动一次自动对齐抓取',command=lambda: self.worker.send(b'l')).pack(side='left',padx=12)
        ttk.Button(actions,text='停止小车 / 中止抓取',command=lambda: self.worker.send(b'x')).pack(side='left',padx=12)
        ttk.Button(actions,text='检测机械臂连接',command=lambda: self.worker.send(b'i')).pack(side='left',padx=12)
        self.car_status=tk.StringVar(value='小车抓取状态：等待遥测')
        ttk.Label(root,textvariable=self.car_status,wraplength=960).pack()
        ttk.Label(root,textvariable=self.status,font=('Microsoft YaHei',11),wraplength=960).pack(padx=12,pady=12)
        ttk.Label(root,text='绿色框：物块候选；青色框：记忆辅助跟踪；黄色框：形状待确认。\n稳定后记忆保留 2 秒，最多辅助 2 帧；完全丢失时不显示成功框。').pack(pady=5)
        self.tracker = Tracker(); self.frame = None; self.result = []; self.last_frame = 0
        self.last_settings = None
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50,self.poll)

    def render(self, frame):
        now=time.monotonic()
        if (frame.width,frame.height)!=(160,120) or not frame.flags & RAW_FLAG:
            self.status.set('等待 160×120 无标记图像；当前帧不参与识别')
            return
        settings=(self.dominance.get(),self.area.get())
        if settings != self.last_settings:
            self.tracker=Tracker(); self.last_settings=settings
        mask, self.result=detect(frame.pixels,dominance=settings[0],minimum_area=settings[1])
        count=self.tracker.update(self.result,frame.sequence,now)
        if self.tracker.target is not None:
            self.result.sort(key=lambda c: c is self.tracker.target, reverse=True)
        self.frame=frame
        fps=1/(now-self.last_frame) if self.last_frame else 0
        self.last_frame=now
        rgb=b''.join(RGB332_TABLE[v] for v in frame.pixels)
        self.photo=tk.PhotoImage(data=b'P6\n160 120\n255\n'+rgb,format='PPM').zoom(4)
        self.mask_photo=tk.PhotoImage(data=b'P6\n160 120\n255\n'+b''.join(b'\xff\xff\xff' if v else b'\0\0\0' for v in mask),format='PPM').zoom(2)
        self.canvas.delete('all'); self.mask_canvas.delete('all')
        self.canvas.create_image(0,0,anchor='nw',image=self.photo)
        self.mask_canvas.create_image(0,0,anchor='nw',image=self.mask_photo)
        self.canvas.create_line(320,0,320,480,fill='#aaaaaa',dash=(3,5))
        for i,c in enumerate(self.result[:8]):
            l,t,r,b=c['box']; x,y=c['center']; color='#00ff66' if c['accepted'] else '#ffcc00'
            assisted = c is self.tracker.target and self.tracker.assisted
            if assisted: color='#00e5ff'
            self.canvas.create_rectangle(l*4,t*4,(r+1)*4,(b+1)*4,outline=color,width=2)
            self.canvas.create_polygon(*[v*4 for p in c['polygon'] for v in p],outline=color,fill='',dash=(3,3))
            self.canvas.create_line(x*4-8,y*4,x*4+8,y*4,fill=color,width=2)
            self.canvas.create_line(x*4,y*4-8,x*4,y*4+8,fill=color,width=2)
            self.canvas.create_text(l*4,max(10,t*4-10),text=f'{i+1} {"记忆辅助" if assisted else c["reason"]}',fill=color,anchor='w')
        if self.result:
            c=self.result[0]; l,t,r,b=c['box']; x,y=c['center']
            state = '稳定候选（记忆辅助）' if self.tracker.assisted else ('稳定物块候选' if count>=3 else c['reason'])
            self.status.set(f'{state} · 跟踪计数 {count} | 中心 ({x:.1f}, {y:.1f}) | 偏差 {x-79.5:+.1f} px | 尺寸 {r-l+1}×{b-t+1} px | 深绿占比 {c["dark_support"]:.0%} | 记忆 {self.tracker.memory_remaining(now):.1f} 秒 | {fps:.1f} FPS')
        else:
            self.status.set(f'未找到绿色物块候选 | 记忆剩余 {self.tracker.memory_remaining(now):.1f} 秒 | {fps:.1f} FPS')

    def poll(self):
        latest=None
        while True:
            try: kind,value=self.messages.get_nowait()
            except queue.Empty: break
            if kind=='frame': latest=value
            elif kind=='line' and value.startswith('STATUS '):
                fields=dict(p.split('=',1) for p in value.split() if '=' in p)
                self.car_status.set('小车模式：%s　超声原始/滤波：%s mm　电机：%s' %
                                    (fields.get('mode','?'),fields.get('us','?'),fields.get('cmd','?')))
            elif kind=='line' and value.startswith('EVENT ') and ('arm_' in value or 'cube_' in value):
                self.car_status.set(value)
            elif kind in ('error','connection'): self.status.set(str(value))
        if latest: self.render(latest)
        if self.last_frame and time.monotonic()-self.last_frame>2:
            self.status.set('画面已过期，等待摄像头恢复'); self.tracker=Tracker()
        self.root.after(50,self.poll)

    def save(self):
        if not self.frame or time.monotonic()-self.last_frame>2: return
        directory=Path(__file__).resolve().parent/'cube-captures'; directory.mkdir(exist_ok=True)
        base=directory/time.strftime('%Y%m%d-%H%M%S')
        base.with_suffix('.ppm').write_bytes(b'P6\n160 120\n255\n'+b''.join(RGB332_TABLE[v] for v in self.frame.pixels))
        base.with_suffix('.json').write_text(json.dumps(dict(sequence=self.frame.sequence,candidates=self.result,tracked_target=self.tracker.target,memory_assisted=self.tracker.assisted,tracking_count=self.tracker.count,dominance=self.dominance.get(),minimum_area=self.area.get()),ensure_ascii=False,indent=2),encoding='utf-8')
        self.status.set(f'已保存：{base}')

    def close(self):
        self.worker.close(); self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--port',default='COM3')
    args=parser.parse_args()
    connection=connect_preview(args.port,b'o')
    messages=queue.Queue(maxsize=100)
    worker=SerialWorker(args.port,connection,messages,preview_command=b'o'); worker.start()
    try:
        root=tk.Tk(); Window(root,worker,messages); root.mainloop()
    finally: worker.close()


if __name__=='__main__': main()
