import multiprocessing
import unittest
from robot_heartbeat import SharedHeartbeat

def update_heartbeat(fields):
    fields['last']=123.5
    fields.update(ready=True,operator_live=True)

class SharedHeartbeatTests(unittest.TestCase):
    def test_receiver_process_updates_parent(self):
        ctx=multiprocessing.get_context('spawn')
        fields=SharedHeartbeat(ctx,{'last':0.,'stop':False,'ready':False,'operator_live':False})
        worker=ctx.Process(target=update_heartbeat,args=(fields,))
        worker.start();worker.join(5)
        if worker.is_alive():worker.terminate();worker.join();self.fail('Receiver did not exit')
        self.assertEqual(worker.exitcode,0)
        self.assertEqual(fields['last'],123.5)
        self.assertTrue(fields.get('ready'))
        self.assertTrue(fields['operator_live'])
        self.assertEqual(fields.get('unknown','fallback'),'fallback')
