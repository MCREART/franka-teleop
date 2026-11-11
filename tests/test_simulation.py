"""Safety and contact regression tests against the real MuJoCo model."""
import math
import unittest

import mujoco
import numpy as np

from server.simulation import Simulation


def control(seq=0, *, position=None, quaternion=None, gripper=1.0, engaged=True):
    return {
        "type": "control",
        "seq": seq,
        "position": [0.0, 0.0, 0.0] if position is None else position,
        "quaternion": [0.0, 0.0, 0.0, 1.0] if quaternion is None else quaternion,
        "gripper": gripper,
        "engaged": engaged,
    }


class SimulationControlTests(unittest.TestCase):
    def setUp(self):
        self.sim = Simulation()

    def test_first_clutch_is_relative_and_phone_axes_map_to_robot(self):
        initial = np.array(self.sim.state()["ee_position"])
        initial_rotation = self.sim.target_rotation.copy()
        reference_quaternion = [0.0, math.sin(0.3), 0.0, math.cos(0.3)]
        self.sim.apply_control(
            control(position=[1.0, 2.0, 3.0], quaternion=reference_quaternion), now=10.0
        )
        np.testing.assert_allclose(self.sim.target_position, initial, atol=1e-6)
        np.testing.assert_allclose(self.sim.target_rotation, initial_rotation, atol=1e-6)
        self.sim.apply_control(
            control(1, position=[1.01, 2.02, 2.97], quaternion=reference_quaternion), now=10.03
        )
        np.testing.assert_allclose(
            self.sim.target_position - initial, [-0.01, -0.03, 0.02], atol=1e-6
        )

    def test_side_grip_phone_translation_axes(self):
        self.sim.apply_control(control(), now=10.0)
        initial = self.sim.target_position.copy()
        cases = [
            ([0.03, 0.0, 0.0], [-0.03, 0.0, 0.0]),
            ([0.0, 0.03, 0.0], [0.0, 0.0, 0.03]),
            ([0.0, 0.0, 0.03], [0.0, 0.03, 0.0]),
        ]
        for sequence, (phone_delta, robot_delta) in enumerate(cases, start=1):
            with self.subTest(phone_delta=phone_delta):
                self.sim.apply_control(control(sequence, position=phone_delta), now=10.0 + 0.03 * sequence)
                np.testing.assert_allclose(self.sim.target_position - initial, robot_delta, atol=1e-6)

    def test_malformed_control_does_not_partially_update_state(self):
        cases = [
            {"position": [0, 0]},
            {"position": [0, 0, float("nan")]},
            {"position": [0, 0, float("inf")]},
            {"quaternion": [0, 0, 0, 0]},
            {"quaternion": [0, 0, 1]},
            {"gripper": -0.1},
            {"gripper": 1.1},
            {"gripper": float("nan")},
            {"engaged": "false"},
            {"seq": True},
            {"type": "move"},
        ]
        self.sim.apply_control(control(gripper=0.4), now=10.0)
        position = self.sim.target_position.copy()
        for changes in cases:
            with self.subTest(changes=changes):
                frame = control(1, position=[0.1, 0.2, 0.3], gripper=0.9)
                frame.update(changes)
                with self.assertRaises(ValueError):
                    self.sim.apply_control(frame, now=10.01)
                np.testing.assert_array_equal(self.sim.target_position, position)
                self.assertAlmostEqual(self.sim.gripper_command, 0.4)

    def test_stale_sequence_cannot_override_newer_command(self):
        self.sim.apply_control(control(4, gripper=0.3), now=10.0)
        with self.assertRaises(ValueError):
            self.sim.apply_control(control(3, gripper=1.0), now=10.01)
        self.assertAlmostEqual(self.sim.gripper_command, 0.3)

    def test_side_grip_phone_rotation_axes(self):
        self.sim.apply_control(control(), now=10.0)
        initial = self.sim.target_rotation.copy()
        angle = 0.1
        cosine, sine = math.cos(angle), math.sin(angle)
        expected_rotations = [
            [[1, 0, 0], [0, cosine, sine], [0, -sine, cosine]],  # AR +X → robot -X
            [[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]],  # AR +Y → robot +Z
            [[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]],  # AR +Z → robot +Y
        ]
        for axis, expected_delta in enumerate(expected_rotations):
            with self.subTest(phone_axis=axis):
                quaternion = [0.0, 0.0, 0.0, math.cos(angle / 2)]
                quaternion[axis] = math.sin(angle / 2)
                self.sim.apply_control(control(axis + 1, quaternion=quaternion), now=10.03 + 0.03 * axis)
                np.testing.assert_allclose(
                    self.sim.target_rotation, np.array(expected_delta) @ initial, atol=1e-6
                )

    def test_releasing_clutch_and_reengaging_does_not_jump(self):
        self.sim.apply_control(control(), now=10.0)
        self.sim.apply_control(control(1, position=[0.0, 0.02, 0.0]), now=10.03)
        self.sim.step(dt=0.02, now=10.05)
        self.sim.apply_control(control(2, position=[3.0, 4.0, 5.0], engaged=False), now=10.06)
        self.assertFalse(self.sim.tracking_active)
        resting = np.array(self.sim.state()["ee_position"])
        self.sim.apply_control(control(3, position=[3.0, 4.0, 5.0]), now=10.09)
        np.testing.assert_allclose(self.sim.target_position, resting, atol=1e-6)

    def test_timeout_and_disconnect_hold_gripper(self):
        self.sim.apply_control(control(gripper=0.25), now=10.0)
        self.sim.step(dt=0.01, now=10.6)
        self.assertFalse(self.sim.tracking_active)
        self.assertAlmostEqual(self.sim.gripper_command, 0.25)
        self.sim.apply_control(control(1, gripper=0.3), now=10.61)
        self.sim.disconnect()
        self.assertFalse(self.sim.tracking_active)
        self.assertAlmostEqual(self.sim.gripper_command, 0.3)

    def test_no_grasp_event_when_closing_in_free_space(self):
        self.sim.apply_control(control(gripper=0.0, engaged=False), now=10.0)
        for i in range(150):
            self.sim.step(dt=0.01, now=10.0 + 0.01 * i)
        self.assertFalse(self.sim.state()["grasped"])
        self.assertFalse(any(event.get("grasped") for event in self.sim.pop_events()))

    def test_one_finger_contact_does_not_report_grasp(self):
        # Isolate unilateral contact from falling. The pickup test below keeps normal gravity.
        self.sim.model.opt.gravity[:] = 0
        address = self.sim.model.joint("cube_joint").qposadr[0]
        self.sim.data.qpos[address:address + 3] = (
            self.sim.data.site_xpos[self.sim.site_id] + [0, 0.075, 0]
        )
        mujoco.mj_forward(self.sim.model, self.sim.data)
        maximum_forces = np.zeros(2)
        contact_force = np.zeros(6)
        for i in range(100):
            self.sim.step(dt=0.01, now=10.0 + 0.01 * i)
            forces = np.zeros(2)
            for index in range(self.sim.data.ncon):
                contact = self.sim.data.contact[index]
                if self.sim.cube_geom not in (contact.geom1, contact.geom2):
                    continue
                other = contact.geom2 if contact.geom1 == self.sim.cube_geom else contact.geom1
                body = self.sim.model.geom_bodyid[other]
                if body in self.sim.finger_bodies:
                    mujoco.mj_contactForce(self.sim.model, self.sim.data, index, contact_force)
                    forces[self.sim.finger_bodies.index(body)] += max(0, contact_force[0])
            maximum_forces = np.maximum(maximum_forces, forces)
        self.assertGreater(maximum_forces[1], 0.3)
        self.assertEqual(maximum_forces[0], 0)
        self.assertFalse(self.sim.grasped)
        self.assertEqual(self.sim.pop_events(), [])

    def test_real_cube_pickup_lift_and_release_emits_one_event_each(self):
        """Use normal control frames, gravity, friction, IK and actual bilateral contacts."""
        initial = np.array(self.sim.state()["ee_position"])
        sequence = 0

        def move_to(position, gripper, duration):
            nonlocal sequence
            delta = np.asarray(position) - initial
            phone_position = [float(-delta[0]), float(delta[2]), float(delta[1])]
            for _ in range(round(duration / 0.01)):
                now = 10.0 + sequence * 0.01
                self.sim.apply_control(control(sequence, position=phone_position, gripper=gripper), now=now)
                self.sim.step(dt=0.01, now=now)
                sequence += 1

        move_to(initial, 1.0, 0.3)
        move_to([0.5, 0.0, 0.127], 1.0, 4.5)
        self.assertFalse(self.sim.grasped)
        move_to([0.5, 0.0, 0.127], 0.0, 1.5)
        self.assertTrue(self.sim.grasped)
        events = self.sim.pop_events()
        self.assertEqual([event["grasped"] for event in events], [True])
        self.assertGreater(events[0]["force"], 0.3)

        move_to([0.5, 0.0, 0.30], 0.0, 3.0)
        self.assertGreater(self.sim.data.body("cube").xpos[2], 0.27)
        self.assertTrue(self.sim.grasped)
        self.assertEqual(self.sim.pop_events(), [])

        move_to([0.5, 0.0, 0.30], 1.0, 1.2)
        self.assertFalse(self.sim.grasped)
        self.assertLess(self.sim.data.body("cube").xpos[2], 0.14)
        self.assertEqual([event["grasped"] for event in self.sim.pop_events()], [False])


if __name__ == "__main__":
    unittest.main()
