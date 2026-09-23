import pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).parents[1]/"protocol"))
from reference import *

class ProtocolTests(unittest.TestCase):
    def test_crc_vector(self): self.assertEqual(crc16(b"123456789"),0x29B1)
    def test_fragment_and_noise(self):
        p=StreamParser(); f1=frame("P1,GET"); f2=frame("P1,HB,7")
        self.assertEqual(p.feed(b"noise"+f1[:4]),[])
        self.assertEqual(p.feed(f1[4:]+f2),[f1,f2])
    def test_bad_crc(self):
        good=frame("P1,GET"); self.assertTrue(verify(good))
        self.assertFalse(verify(good[:-5]+b"0000#"))
    def test_prepare_atomic_six(self):
        raw=prepare(9,5000,Pid(97.0,48.5,62,31,17,20))
        self.assertTrue(verify(raw)); self.assertIn(b",62.00,31.00,17.00,20.00,",raw)
    def test_manual_and_training_frames(self):
        self.assertTrue(verify(manual_set(Pid(96,48,62,31,17,20))))
        self.assertTrue(verify(frame("P1,TRAIN,START")))
    def test_motor_diagnostic_frames(self):
        for action in ("L+","L-","R+","R-","STOP"):
            self.assertTrue(verify(motor_diag(action)))
    def test_legacy_coexists(self):
        p=StreamParser(); legacy=b"$0,0,1,0,0,0,0#"
        self.assertEqual(p.feed(legacy),[legacy]); self.assertFalse(verify(legacy))

if __name__=="__main__": unittest.main()
