from __future__ import annotations

import threading
import unittest

from route_a.ui import RouteAUI


class ImmediateRoot:
    def after(self,delay,callback): callback()


class UIWorkerTests(unittest.TestCase):
    def test_delayed_tk_callback_retains_exception_and_traceback(self):
        ui=object.__new__(RouteAUI); ui.busy=False; ui.root=ImmediateRoot()
        completed=threading.Event(); captured={}
        def done(value,error,details=""):
            captured.update(value=value,error=error,details=details); completed.set()
        ui._worker_done=done
        def fail(): raise RuntimeError("worker failure")
        ui._worker(fail)
        self.assertTrue(completed.wait(2)); self.assertIsInstance(captured["error"],RuntimeError)
        self.assertIn("worker failure",captured["details"])


if __name__=="__main__": unittest.main()
