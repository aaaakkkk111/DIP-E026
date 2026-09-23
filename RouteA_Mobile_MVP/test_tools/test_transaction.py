import unittest
from dataclasses import dataclass

@dataclass
class Txn:
    champion: tuple=(96.0,48.0,62.0,31.0,17.0,20.0)
    current: tuple=(96.0,48.0,62.0,31.0,17.0,20.0)
    candidate: tuple|None=None
    state: str="IDLE"
    training_locked: bool=False
    def manual_set(self,p):
        assert not self.training_locked and self.state=="IDLE"
        self.current=p;self.champion=p
    def start_training(self):
        assert self.state=="IDLE";self.champion=self.current;self.training_locked=True
    def stop_training(self):
        if self.state!="IDLE":self.rollback()
        self.training_locked=False
    def prepare(self,p):
        assert self.training_locked and self.state=="IDLE" and p[2:]==self.champion[2:]
        assert all(abs(p[i]-self.champion[i])*100/self.champion[i]<=5.01 for i in (0,1))
        self.candidate=p;self.state="PREPARED"
    def apply(self): assert self.state=="PREPARED";self.current=self.candidate;self.state="TRIAL"
    def rollback(self): self.current=self.champion;self.candidate=None;self.state="IDLE"
    def accept(self): assert self.state=="TRIAL";self.champion=self.current;self.candidate=None;self.state="IDLE"

class TransactionTests(unittest.TestCase):
    def test_apply_rollback_is_atomic(self):
        t=Txn();t.start_training();p=(97.0,48.5,62.0,31.0,17.0,20.0);t.prepare(p);self.assertEqual(t.current,t.champion);t.apply();self.assertEqual(t.current,p);t.rollback();self.assertEqual(t.current,t.champion)
    def test_accept_promotes_champion(self):
        t=Txn();t.start_training();p=(95.0,47.5,62.0,31.0,17.0,20.0);t.prepare(p);t.apply();t.accept();self.assertEqual(t.champion,p)
    def test_rejects_hidden_loop_change(self):
        t=Txn();t.start_training()
        with self.assertRaises(AssertionError):t.prepare((96,48,63,31,17,20))
    def test_rejects_large_change(self):
        t=Txn();t.start_training()
        with self.assertRaises(AssertionError):t.prepare((110,48,62,31,17,20))
    def test_manual_before_training_and_lock_after_start(self):
        t=Txn();manual=(96,48,62,31,17,20);t.manual_set(manual);t.start_training()
        self.assertEqual(t.champion,manual)
        with self.assertRaises(AssertionError):t.manual_set((97,48,62,31,17,20))
        t.stop_training();t.manual_set((97,48,62,31,17,20))
    def test_candidate_rejected_before_training(self):
        with self.assertRaises(AssertionError):Txn().prepare((97,48,62,31,17,20))
    def test_complete_training_accept_and_unlock_cycle(self):
        t=Txn()
        baseline=(96.0,48.0,62.0,31.0,17.0,20.0)
        t.manual_set(baseline)
        t.start_training()
        candidate=(99.84,49.92,62.0,31.0,17.0,20.0)
        t.prepare(candidate)
        self.assertEqual(t.state,"PREPARED")
        t.apply()
        self.assertEqual(t.current,candidate)
        t.accept()
        self.assertEqual(t.champion,candidate)
        t.stop_training()
        self.assertFalse(t.training_locked)
        self.assertEqual(t.state,"IDLE")

if __name__=="__main__":unittest.main()
