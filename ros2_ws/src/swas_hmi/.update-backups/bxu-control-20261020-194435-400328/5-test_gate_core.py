import unittest
from gate_core import GateCore

class GateTests(unittest.TestCase):
    def ready(self):
        gate = GateCore()
        for k,v in [('clock',1.0),('mission','WAITING'),('front',False),('dock',False),('lift','LOWERED')]:
            gate.observe(k,v,10.0)
        return gate

    def test_starts_stopped(self):
        self.assertEqual(GateCore().output(10), (0.0,0.0,0.0))

    def test_manual_timeout_never_falls_back_to_auto(self):
        g=self.ready()
        self.assertTrue(g.select('MANUAL',10)[0])
        g.command('manual',(.04,0,0),10)
        g.command('auto',(.15,0,0),10.4)
        self.assertEqual(g.output(10.1),(.04,0,0))
        self.assertEqual(g.output(10.4),(0.0,0.0,0.0))
        self.assertEqual(g.mode,'MANUAL')

    def test_paused_clock_latches_stop(self):
        g=self.ready();g.select('AUTO',10);g.command('auto',(.1,0,0),10.7)
        self.assertEqual(g.output(10.7),(0.0,0.0,0.0));self.assertEqual(g.mode,'STOP')
        g.observe('clock',2.0,10.8)
        self.assertEqual(g.output(10.8),(0.0,0.0,0.0))

    def test_active_lift_refuses_manual(self):
        g=self.ready();g.observe('lift','LIFTING',10)
        self.assertFalse(g.select('MANUAL',10)[0])

    def test_mode_change_discards_previous_commands(self):
        g=self.ready();g.command('manual',(.04,0,0),10)
        g.select('MANUAL',10)
        self.assertEqual(g.output(10),(0.0,0.0,0.0))

    def test_nonfinite_stops(self):
        g=self.ready();g.select('MANUAL',10);g.command('manual',(float('nan'),0,0),10)
        self.assertEqual(g.mode,'STOP')

    def test_limits(self):
        g=self.ready();g.select('MANUAL',10);g.command('manual',(9,-9,9),10)
        self.assertEqual(g.output(10),(.05,-.05,.15))

if __name__ == '__main__':
    unittest.main()
