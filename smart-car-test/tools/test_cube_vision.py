import unittest
from pathlib import Path
from cube_grab_monitor import detect, Tracker


class CubeTests(unittest.TestCase):
    def test_calibrated_real_angles(self):
        for name, expected in (('cube-angle-original.rgb332',(77,74)),
                               ('cube-angle-side.rgb332',(77,38)),
                               ('cube-angle-side-dark.rgb332',(77,38)),
                               ('cube-far-dark.rgb332',(79,21))):
            pixels=(Path(__file__).parent/'test-data'/name).read_bytes()
            candidates=detect(pixels)[1]
            accepted=[c for c in candidates if c['accepted']]
            self.assertEqual(len(accepted),1,name)
            self.assertAlmostEqual(accepted[0]['center'][0],expected[0],delta=4)
            self.assertAlmostEqual(accepted[0]['center'][1],expected[1],delta=4)

    def test_dark_face_repair_without_accepting_thin_green_outline(self):
        for inset,expected in ((8,True),(2,False)):
            pixels=self.image(lambda x,y: 40<=x<70 and 40<=y<70 and not
                              (40+inset<=x<70-inset and 40+inset<=y<70-inset))
            self.assertEqual(any(c['accepted'] for c in detect(pixels)[1]),expected)

    def test_dark_quantized_green_and_small_far_target(self):
        for color in (0x29,0x04):
            pixels=self.image(lambda x,y: 70<=x<77 and 20<=y<27,color)
            candidates=detect(pixels)[1]
            self.assertEqual(len(candidates),1)
            self.assertTrue(candidates[0]['accepted'])

    def test_bright_green_gray_blue_and_black_rejected(self):
        for color in (0xbd,0xff,0x92,0x2b,0x00):
            pixels=self.image(lambda x,y: 40<=x<70 and 40<=y<70,color)
            self.assertFalse(detect(pixels)[1],hex(color))

    def test_light_noise_with_tiny_dark_seed_rejected(self):
        p=bytearray(self.image(lambda x,y: 40<=x<70 and 40<=y<70,0x71))
        for y in range(50,52):
            for x in range(50,52): p[y*160+x]=0x29
        self.assertFalse(detect(p)[1])

    def stable_tracker(self):
        c=detect(self.image(lambda x,y: 40<=x<70 and 40<=y<70))[1][0]
        t=Tracker()
        for seq in range(3): t.update([c],seq,seq*0.6)
        weak=dict(c,accepted=False,reason='形状待确认',fill=0.6,solidity=0.85)
        return t,c,weak

    def test_memory_recovers_after_missing_frame(self):
        t,c,weak=self.stable_tracker()
        self.assertEqual(t.update([],3,1.8),0)
        self.assertIsNone(t.target)
        self.assertEqual(t.update([weak],4,2.4),3)
        self.assertTrue(t.assisted)
        self.assertFalse(weak['accepted'])
        self.assertEqual(t.update([weak],4,2.5),3)
        self.assertEqual(t.assisted_frames,1)
        t.update([c],5,3.0)
        self.assertFalse(t.assisted)
        self.assertEqual(t.anchor_time,3.0)

    def test_memory_does_not_self_refresh(self):
        t,c,weak=self.stable_tracker()
        t.update([weak],3,1.8)
        t.update([weak],4,2.4)
        self.assertTrue(t.assisted)
        self.assertEqual(t.update([weak],5,2.9),0)
        self.assertEqual(t.update([weak],6,3.3),0)
        self.assertIsNone(t.anchor)

    def test_memory_requires_stable_target_and_geometry(self):
        _,c,weak=self.stable_tracker()
        t=Tracker(); t.update([c],1,0)
        self.assertEqual(t.update([weak],2,0.6),0)
        for change in (dict(center=[90,55]),dict(area=100),dict(box=[40,40,100,69]),
                       dict(reason='边缘截断'),dict(solidity=0.3)):
            t,_,weak=self.stable_tracker()
            self.assertEqual(t.update([dict(weak,**change)],3,1.8),0)

    def test_near_memory_preferred_to_distant_larger_blob(self):
        t,c,weak=self.stable_tracker()
        distant=dict(c,center=[120,55],box=[100,40,139,79],area=1600)
        t.update([distant,weak],3,1.8)
        self.assertIs(t.target,weak)
        self.assertTrue(t.assisted)

    def image(self, predicate, color=0x2c):
        return bytes(color if predicate(x,y) else 0 for y in range(120) for x in range(160))

    def test_square_and_rotated_square(self):
        for shape in (lambda x,y: 40<=x<70 and 40<=y<70,
                      lambda x,y: abs(x-70)+abs(y-60)<=22):
            _, candidates=detect(self.image(shape))
            self.assertTrue(candidates[0]['accepted'])

    def test_circle_and_strip_rejected(self):
        for shape in (lambda x,y: (x-70)**2+(y-60)**2<=20**2,
                      lambda x,y: 40<=x<100 and 40<=y<46):
            _, candidates=detect(self.image(shape))
            self.assertFalse(candidates[0]['accepted'])

    def test_perspective_face(self):
        _, candidates=detect(self.image(lambda x,y: 30<=y<=70 and
            60-(y-30)*0.25<=x<=80+(y-30)*0.25))
        self.assertTrue(candidates[0]['accepted'])

    def test_wrong_color_and_noise(self):
        self.assertFalse(detect(bytes(19200))[1])
        self.assertFalse(detect(self.image(lambda x,y: 40<x<70 and 40<y<70,0xe0))[1])
        self.assertFalse(detect(self.image(lambda x,y: x%10==0 and y%10==0))[1])

    def test_tracking_loss_duplicate_and_timeout(self):
        c=detect(self.image(lambda x,y: 40<=x<70 and 40<=y<70))[1]
        t=Tracker()
        self.assertEqual(t.update(c,1,1),1)
        self.assertEqual(t.update(c,1,1.1),1)
        self.assertEqual(t.update(c,2,1.6),2)
        self.assertEqual(t.update(c,3,2.2),3)
        self.assertEqual(t.update(c,4,5),1)
        self.assertEqual(t.update([],5,5.6),0)


if __name__=='__main__': unittest.main()
