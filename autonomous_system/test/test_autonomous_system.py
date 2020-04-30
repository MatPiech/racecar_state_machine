import unittest
import sys
sys.path.append('../')
from autonomous_system import AutonomousSystem as AS


class TestAutonomousSystem(unittest.TestCase):
    def test_SM_states_quantity(self):
        SM = AS()
        states_quantity = 6
        self.assertEqual(len([s.identifier for s in SM.states]), states_quantity)

    def test_SM_transitions_quantity(self):
        SM = AS()
        transitions_quantity = 11
        self.assertEqual(len([s.identifier for s in SM.transitions]), transitions_quantity)

    def test_path_off_manual_off(self):
        SM = AS()
        path = ['off_manual', 'manual_off']
        states = [SM.manual_driving, SM.as_off]
        for i, transition in enumerate(path):
            SM.transitions_map[transition]._run(SM)
            self.assertEqual(SM.current_state, states[i])

    def test_path_off_ready_off(self):
        SM = AS()
        path = ['off_ready', 'ready_off']
        states = [SM.as_ready, SM.as_off]
        for i, transition in enumerate(path):
            SM.transitions_map[transition]._run(SM)
            self.assertEqual(SM.current_state, states[i])

    def test_path_off_ready_emergency_off(self):
        SM = AS()
        path = ['off_ready', 'ready_emergency', 'emergency_off']
        states = [SM.as_ready, SM.as_emergency, SM.as_off]
        for i, transition in enumerate(path):
            SM.transitions_map[transition]._run(SM)
            self.assertEqual(SM.current_state, states[i])

    def test_path_off_ready_driving_emergency_off(self):
        SM = AS()
        path = ['off_ready', 'ready_driving', 'driving_emergency', 'emergency_off']
        states = [SM.as_ready, SM.as_driving, SM.as_emergency, SM.as_off]
        for i, transition in enumerate(path):
            SM.transitions_map[transition]._run(SM)
            self.assertEqual(SM.current_state, states[i])

    def test_path_off_ready_driving_finished_off(self):
        SM = AS()
        path = ['off_ready', 'ready_driving', 'driving_finished', 'finished_off']
        states = [SM.as_ready, SM.as_driving, SM.as_finished, SM.as_off]
        for i, transition in enumerate(path):
            SM.transitions_map[transition]._run(SM)
            self.assertEqual(SM.current_state, states[i])

    def test_path_off_ready_driving_finished_emergency_off(self):
        SM = AS()
        path = ['off_ready', 'ready_driving', 'driving_finished', 'finished_emergency', 'emergency_off']
        states = [SM.as_ready, SM.as_driving, SM.as_finished, SM.as_emergency, SM.as_off]
        for i, transition in enumerate(path):
            SM.transitions_map[transition]._run(SM)
            self.assertEqual(SM.current_state, states[i])

    def test_states_transitions_quantity(self):
        SM = AS()
        transitions = {'as_off': 2, 'manual_driving': 1, 'as_ready': 3, 'as_driving': 2, 'as_finished': 2, 'as_emergency': 1}
        self.assertEqual(len(SM.as_off.transitions), transitions['as_off'])
        self.assertEqual(len(SM.manual_driving.transitions), transitions['manual_driving'])
        self.assertEqual(len(SM.as_ready.transitions), transitions['as_ready'])
        self.assertEqual(len(SM.as_driving.transitions), transitions['as_driving'])
        self.assertEqual(len(SM.as_finished.transitions), transitions['as_finished'])
        self.assertEqual(len(SM.as_emergency.transitions), transitions['as_emergency'])


if __name__ == '__main__':
    unittest.main()
