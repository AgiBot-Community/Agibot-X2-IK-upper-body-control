"""Offline regression tests for the installed solver and application entry points."""
import io
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

from fk_oracle import IndependentFK, errors
from x2ik.x2_api import X2Arm, HOME
from x2ik.x2_arm_dynamics import ArmDynamics
from x2ik.x2_arm_model import ArmModel, DEFAULT_URDF, matrix_to_rpy
from x2ik.x2_mdi import PureIKWorker, _solve_pose
from x2ik.x2_mdi_bridge import Bridge, Transport
from x2ik.x2_srs_ik import SrsArmIK, IKSolution, branch_tuple


class SolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures = json.loads((Path(__file__).parent / 'fixtures/wrist_regression.json').read_text())
        cls.oracles = {s: IndependentFK(DEFAULT_URDF.read_bytes(), s) for s in ('right', 'left')}

    def check_solution(self, sol, pose, oracle):
        self.assertIsInstance(sol, IKSolution)
        actual = oracle.pose(sol.q)
        pe, re = errors(actual, pose)
        self.assertLessEqual(pe, 0.01)
        self.assertLessEqual(re, 0.001)
        self.assertTrue(np.all(sol.q >= oracle.low - 1e-9))
        self.assertTrue(np.all(sol.q <= oracle.high + 1e-9))
        self.assertTrue(sol.in_limits)
        self.assertAlmostEqual(sol.pos_error * 1000, pe, places=8)
        self.assertAlmostEqual(math.degrees(sol.rot_error), re, places=7)

    def test_all_baseline_targets_without_known_seed(self):
        for side in ('right', 'left'):
            solver = SrsArmIK(ArmModel(side))
            for row in self.fixtures['arms'][side]:
                with self.subTest(side=side, index=row['index']):
                    pose = (np.array(row['position_m']), np.array(row['rotation']))
                    self.check_solution(solver.solve(*pose), pose, self.oracles[side])

    def test_fresh_targets_without_known_seed(self):
        rng = np.random.default_rng(20260928)
        for side, oracle in self.oracles.items():
            solver = SrsArmIK(ArmModel(side))
            for index, q in enumerate(rng.uniform(oracle.low, oracle.high, (100, 7))):
                with self.subTest(side=side, index=index):
                    pose = oracle.pose(q)
                    self.check_solution(solver.solve(*pose), pose, oracle)

    def test_existing_pose_keeps_joint_angles_and_legacy_fields(self):
        for side, oracle in self.oracles.items():
            solver = SrsArmIK(ArmModel(side))
            pose = oracle.pose(HOME)
            sol = solver.solve(*pose, q_seed=HOME)
            self.check_solution(sol, pose, oracle)
            np.testing.assert_array_equal(sol.q, HOME)
            self.assertEqual(sol.source, 'keep_seed')
            self.assertTrue(math.isfinite(sol.psi))
            self.assertEqual(len(branch_tuple(sol)), 3)
            self.assertFalse(np.shares_memory(sol.q, HOME))

    def test_solve_continuous_path_with_step_bound(self):
        for side, oracle in self.oracles.items():
            solver = SrsArmIK(ArmModel(side))
            previous = HOME.copy()
            direction = np.array([.12, .10 if side == 'left' else -.10, .08, -.12, .08, .08, -.08])
            for i, t in enumerate(np.linspace(0, 2 * np.pi, 121)):
                with self.subTest(side=side, frame=i):
                    pose = oracle.pose(HOME + np.sin(t) * direction)
                    sol = solver.solve(*pose, q_seed=previous, max_joint_step_rad=math.radians(5))
                    self.check_solution(sol, pose, oracle)
                    self.assertLessEqual(np.max(np.abs(sol.q - previous)), math.radians(5))
                    previous = sol.q

    def test_tracking_path_preserves_branch_without_global_recovery(self):
        for side, oracle in self.oracles.items():
            solver = SrsArmIK(ArmModel(side))
            previous, psi, branch = HOME.copy(), None, None
            direction = np.array([.12, .10 if side == 'left' else -.10, .08, -.12, .08, .08, -.08])
            with patch.object(solver, 'solve', side_effect=AssertionError('global search in track')):
                for i, t in enumerate(np.linspace(0, 2 * np.pi, 121)):
                    with self.subTest(side=side, frame=i):
                        pose = oracle.pose(HOME + np.sin(t) * direction)
                        sol = solver.track(*pose, q_prev=previous, psi_prev=psi,
                                           branch_prev=branch, fallback=False, clamp_reach=False)
                        self.check_solution(sol, pose, oracle)
                        self.assertLessEqual(np.max(np.abs(sol.q - previous)), 0.05)
                        self.assertTrue(all(v in (0, 1) for v in branch_tuple(sol)))
                        if branch is not None:
                            self.assertEqual(branch_tuple(sol), branch)
                        previous, psi, branch = sol.q, sol.psi, branch_tuple(sol)

    def test_unreachable_requests_are_rejected(self):
        for side in ('left', 'right'):
            solver = SrsArmIK(ArmModel(side))
            for axis in np.eye(3):
                for sign in (-1, 1):
                    pos = axis * sign * 10
                    self.assertIsNone(solver.solve(pos, np.eye(3)))
                    self.assertIsNone(solver.track(pos, np.eye(3), HOME, fallback=False, clamp_reach=False))

    def test_step_limit_rejects_large_motion(self):
        solver = SrsArmIK(ArmModel('left'))
        target = HOME.copy()
        target[0] += .5
        pose = self.oracles['left'].pose(target)
        self.assertIsNone(solver.solve(*pose, q_seed=HOME, max_joint_step_rad=1e-4))
        self.assertIsNone(solver.track(*pose, q_prev=HOME, fallback=False,
                                      clamp_reach=False, max_joint_step_rad=1e-4))

    def test_wrong_branch_and_failed_local_search_are_rejected(self):
        solver = SrsArmIK(ArmModel('left'))
        pose = self.oracles['left'].pose(HOME)
        current = solver.track(*pose, q_prev=HOME, fallback=False, clamp_reach=False)
        wrong = tuple(1 - b for b in branch_tuple(current))
        self.assertIsNone(solver.track(*pose, q_prev=HOME, branch_prev=wrong,
                                      fallback=False, clamp_reach=False))
        target = HOME.copy()
        target[0] += .01
        with patch.object(solver, 'numeric_solve', return_value=None), \
             patch.object(solver, '_track_analytic', return_value=None), \
             patch.object(solver, 'solve', side_effect=AssertionError('global fallback')):
            self.assertIsNone(solver.track(*self.oracles['left'].pose(target), q_prev=HOME,
                                          fallback=False, clamp_reach=False))

    def test_invalid_requests_fail_explicitly(self):
        solver = SrsArmIK(ArmModel('left'))
        p, r = self.oracles['left'].pose(HOME)
        outside = HOME.copy()
        outside[0] = 100
        for pos, rot, kw in [([np.nan, 0, 0], r, {}), (p, np.zeros((3, 3)), {}),
                             (p, np.diag([1, 1, -1]), {}), (p, r, {'q_seed': outside}),
                             (p, r, {'q_seed': [0] * 6}), (p, r, {'max_joint_step_rad': .1}),
                             (p, r, {'q_seed': HOME, 'max_joint_step_rad': float('nan')}),
                             (p, r, {'psi_samples': 0})]:
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                solver.solve(pos, rot, **kw)
        with self.assertRaises(ValueError):
            solver.track(p, r, HOME, branch_prev=(-1, -1, -1))

    def test_legacy_solve_options_and_numeric_opt_out(self):
        solver = SrsArmIK(ArmModel('right'))
        target = HOME + np.array([.05, -.03, .02, -.03, .02, .02, -.02])
        pose = self.oracles['right'].pose(target)
        with patch.object(solver, 'numeric_solve', side_effect=AssertionError('numeric opt-out')):
            sol = solver.solve(*pose, q_seed=HOME, psi_samples=180,
                               psi_hint=solver.sew_angle(HOME), refine=True, polish=True,
                               weight_seed=1., weight_margin=.6, weight_manip=.15,
                               fallback_numeric=False)
            self.check_solution(sol, pose, self.oracles['right'])

    def test_candidate_residuals_are_recomputed_before_return(self):
        solver = SrsArmIK(ArmModel('left'))
        target = HOME.copy()
        target[0] += .1
        pose = self.oracles['left'].pose(target)
        candidate = IKSolution(HOME.copy(), 0., 0, 0, 0, True, 0., 0.)
        with patch.object(solver, '_solve_analytic', return_value=candidate), \
             patch.object(solver, 'numeric_solve', return_value=candidate):
            self.assertIsNone(solver.solve(*pose, q_seed=HOME))

    def test_joint_step_is_not_wrapped_across_a_limit(self):
        solver = SrsArmIK(ArmModel('right'))
        seed, target = HOME.copy(), HOME.copy()
        seed[0], target[0] = solver.model.q_min[0] + .001, solver.model.q_max[0] - .001
        bound = 2 * math.pi - (target[0] - seed[0]) + .01
        self.assertTrue(solver.model.within_limits(seed))
        self.assertTrue(solver.model.within_limits(target))
        pose = self.oracles['right'].pose(target)
        candidate = IKSolution(target.copy(), 0., 0, 0, 0, True)
        with patch.object(solver, '_solve_analytic', return_value=candidate):
            self.assertIsNone(solver.solve(*pose, q_seed=seed,
                                          fallback_numeric=False, max_joint_step_rad=bound))

    def test_wrist_origin_poses(self):
        rng = np.random.default_rng(7101)
        for side, oracle in self.oracles.items():
            model = ArmModel(side)
            solver = SrsArmIK(model)
            for index, q in enumerate(rng.uniform(oracle.low, oracle.high, (12, 7))):
                with self.subTest(side=side, index=index):
                    pose = oracle.pose(q)
                    self.check_solution(solver.solve(*pose), pose, oracle)

    def test_removed_end_effector_tcp_modes_are_rejected(self):
        from x2ik import Robot
        from x2ik.__main__ import main as cli_main
        from x2ik.x2_mdi_bridge import validate_request

        for field, value in (("tcp_mode", "custom"), ("tcp_file", "tool.json"),
                             ("tcp_offset", [0., 0., -.1]), ("payload", .5),
                             ("payload_com", [0., 0., -.1]), ("hand_mode", 2),
                             ("hand_sub_mode", 1), ("hand_pos", [0., 0.])):
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "腕原点"):
                validate_request({"id": 1, "op": "mdi", "mode": "j", "side": "right",
                                  "values": [0.] * 7, field: value})

        for mode in ('hand', 'gripper', 'custom'):
            with self.subTest(mode=mode, entry='Robot'), \
                 patch('x2ik.x2_movej._X2Arm') as arm:
                with self.assertRaises(TypeError):
                    Robot(tcp_mode=mode)
                arm.assert_not_called()
            with self.subTest(mode=mode, entry='Bridge'), \
                 patch.object(Bridge, '_connect') as connect:
                with self.assertRaises(TypeError):
                    Bridge(Transport(io.StringIO(), io.StringIO()), tcp_mode=mode)
                connect.assert_not_called()
            for entry in ('movej', 'mdi'):
                with self.subTest(mode=mode, entry=entry), \
                     patch('sys.stderr', new_callable=io.StringIO) as stderr:
                    with self.assertRaises(SystemExit) as rejected:
                        cli_main([entry, '--tcp-mode', mode])
                    self.assertEqual(rejected.exception.code, 2)
                    self.assertIn('unrecognized arguments', stderr.getvalue())

        with patch('x2ik.x2_movej._X2Arm') as arm:
            with self.assertRaises(TypeError):
                Robot(tcp_file='removed-tool.json')
            arm.assert_not_called()
        with patch.object(Bridge, '_connect') as connect:
            with self.assertRaises(TypeError):
                Bridge(Transport(io.StringIO(), io.StringIO()), tcp_file='removed-tool.json')
            connect.assert_not_called()
        with patch.object(X2Arm, '_connect') as connect:
            for option in ({'tcp_offset': [0., 0., -.1]}, {'payload': .5}):
                with self.subTest(option=next(iter(option))), self.assertRaises(TypeError):
                    X2Arm(**option)
            connect.assert_not_called()
        for option in ({'tcp_offset': [0., 0., -.1]}, {'tcp_rotation': np.eye(3)}):
            with self.subTest(option=next(iter(option))), self.assertRaises(TypeError):
                ArmModel(**option)
        for entry in ('movej', 'mdi'):
            with self.subTest(entry=entry, option='tcp_file'), \
                 patch('sys.stderr', new_callable=io.StringIO) as stderr:
                with self.assertRaises(SystemExit) as rejected:
                    cli_main([entry, '--tcp-file', 'removed-tool.json'])
                self.assertEqual(rejected.exception.code, 2)
                self.assertIn('unrecognized arguments', stderr.getvalue())
        model = ArmModel('right')
        for option in ({'payload_mass': .5}, {'payload_com': [0., 0., -.1]}):
            with self.subTest(option=next(iter(option))), self.assertRaises(TypeError):
                ArmDynamics(model, **option)
        with self.assertRaises(TypeError):
            ArmDynamics(model).gravity_torque(HOME, include_payload=True)
        from x2ik import x2_sim_ros
        for option, value in (('--payload', '.5'), ('--hand-mode', '2')):
            with self.subTest(option=option), \
                 patch('sys.stderr', new_callable=io.StringIO) as stderr:
                with self.assertRaises(SystemExit) as rejected:
                    x2_sim_ros.main([option + '=' + value, 'mdi'])
                self.assertEqual(rejected.exception.code, 2)
                self.assertIn('unrecognized arguments', stderr.getvalue())
        for option in ({'payload': .5}, {'hand_mode': 2}):
            with self.subTest(option=next(iter(option))), self.assertRaises(TypeError):
                x2_sim_ros.X2ArmClient('upper_body', **option)

    def test_default_robot_uses_wrist_origin_without_tcp_configuration(self):
        from x2ik import Robot

        result = {'q': HOME.copy()}
        with patch('x2ik.x2_movej._X2Arm') as arm:
            arm.return_value.R_move_J.return_value = result
            with Robot(verbose=False) as robot:
                for mode in ('hand', 'gripper', 'custom'):
                    with self.subTest(mode=mode), self.assertRaises(TypeError):
                        robot.moveJ(HOME, tcp_mode=mode)
                arm.return_value.R_move_J.assert_not_called()
                self.assertIs(robot.moveJ(HOME, side='right'), result)
            arm.assert_called_once()
            arm.return_value.R_move_J.assert_called_once_with(HOME, duration=8.0, settle=2.0)
            arm.return_value.close.assert_called_once_with()

    def test_original_api_and_mdi_use_integrated_solver(self):
        row = self.fixtures['arms']['left'][5]
        pose = np.array(row['position_m']), np.array(row['rotation'])
        with X2Arm('left', connect=False) as arm:
            q = arm.ik(pose[0], matrix_to_rpy(pose[1]))
            self.assertIsNotNone(q)
            full = arm.ik_full(pose[0], matrix_to_rpy(pose[1]))
            self.check_solution(full, pose, self.oracles['left'])
            held = _solve_pose(arm.model, arm.solver, HOME,
                               *arm.model.forward_kinematics(HOME), 0., False)
            self.assertTrue(held['ok'])
            np.testing.assert_array_equal(held['q'], HOME)
            result = _solve_pose(arm.model, arm.solver, HOME, *pose, 0., False)
            self.assertTrue(result['ok'])
            pe, re = errors(self.oracles['left'].pose(result['q']), pose)
            self.assertLessEqual(pe, .01)
            self.assertLessEqual(re, .001)
            worker = PureIKWorker(arm.models, SrsArmIK)
            try:
                self.assertTrue(worker.submit('left', HOME, *pose, 0., False).result(timeout=10)['ok'])
            finally:
                worker.close()
        bridge = Bridge(Transport(io.StringIO(), io.StringIO()), demo=True)
        request = dict(side='left', mode='pose',
                       values=[*pose[0], *np.degrees(matrix_to_rpy(pose[1]))])
        plan = bridge.plan(request)
        pe, re = errors(self.oracles['left'].pose(plan['q']), pose)
        self.assertLessEqual(pe, .01)
        self.assertLessEqual(re, .001)
        self.assertFalse(any(n == 'rclpy' or n.startswith('aimdk_msgs') for n in sys.modules))


if __name__ == '__main__':
    unittest.main()
