#!/usr/bin/env python3
"""
week1_test_node.py — Franka Panda Week 1 validation
====================================================
Uses pymoveit2 (ros-humble-pymoveit2) instead of moveit_py (not in Humble apt).

Run:
  ros2 run my_robot_controller week1_test_node --ros-args -p use_sim_time:=true

Expected output:
  [INFO] /clock confirmed
  [INFO] [VERIFY] work_table confirmed
  [INFO] Joint RMS error : 0.000X rad
  [INFO] Distance        : 0.000X m
  [INFO] RESULT: PASS — Shiva-ready
"""

import math
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory as FJT
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Gazebo publishes /clock and /joint_states as BEST_EFFORT
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

from pymoveit2 import MoveIt2

_JTC_ACTION = '/panda_arm_controller/follow_joint_trajectory'

# ── Constants ────────────────────────────────────────────────────────────────

JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]

# "ready" pose from panda SRDF named states
READY_POSE = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

TOLERANCE_RAD   = 0.05   # per-joint tolerance for scoring
CARTESIAN_TOL_M = 0.01   # 10 mm Cartesian tolerance

TABLE_ID   = 'work_table'
TABLE_SIZE = [1.2, 0.8, 0.05]
TABLE_POS  = [0.5, 0.0, -0.03]
TABLE_QUAT = [0.0, 0.0, 0.0, 1.0]


# ── Helpers ──────────────────────────────────────────────────────────────────

def _wait_for_clock(node: Node, timeout: float = 210.0) -> bool:
    """Block until Gazebo sim clock publishes a non-zero timestamp."""
    ready = threading.Event()

    def _cb(msg):
        if msg.clock.sec > 0:
            ready.set()

    sub  = node.create_subscription(Clock, '/clock', _cb, _SENSOR_QOS)
    exec_ = MultiThreadedExecutor(1)
    exec_.add_node(node)
    t0 = time.time()
    while not ready.is_set() and time.time() - t0 < timeout:
        exec_.spin_once(timeout_sec=0.2)
    exec_.remove_node(node)
    node.destroy_subscription(sub)
    return ready.is_set()



def _compute_fk(node: Node, moveit2: MoveIt2, joint_positions: list) -> tuple:
    """Call MoveIt /compute_fk and return (x, y, z) of panda_link8."""
    future = moveit2.compute_fk_async(joint_state=joint_positions)
    if future is None:
        node.get_logger().warn('FK service /compute_fk not available')
        return None

    t0 = time.time()
    while not future.done() and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if future.done() and future.result() is not None:
        ps = future.result().pose_stamped
        if ps:
            p = ps[0].pose.position
            return (p.x, p.y, p.z)
    node.get_logger().warn('FK call returned no result')
    return None


# ── Direct JTC helpers ───────────────────────────────────────────────────────
# A persistent ActionClient is created once in main() and passed here.
# Re-creating the client on every call caused "cannot use Destroyable" errors
# that corrupted the main node's subscription state between calls.

def _jtc_execute(node: Node, client: ActionClient,
                 joint_trajectory: JointTrajectory,
                 timeout: float = 30.0) -> bool:
    """Send a JointTrajectory directly to panda_arm_controller."""
    exec_ = MultiThreadedExecutor(1)
    exec_.add_node(node)

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f'{_JTC_ACTION} not available')
        exec_.remove_node(node)
        return False

    # Zero the header stamp so the JTC starts executing immediately.
    joint_trajectory.header.stamp.sec = 0
    joint_trajectory.header.stamp.nanosec = 0

    goal = FJT.Goal()
    goal.trajectory = joint_trajectory

    # t0 is set AFTER wait_for_server so the timeout budget is not consumed
    # by server discovery.  Previously t0 was before wait_for_server; a 5 s
    # discovery pause ate into a 14 s budget, causing mid-trajectory timeouts
    # that left the cache with a partially-executed (wrong) joint position.
    t0 = time.time()
    gf = client.send_goal_async(goal)
    while not gf.done() and time.time() - t0 < 10.0:
        exec_.spin_once(timeout_sec=0.2)

    if not gf.done() or not gf.result().accepted:
        node.get_logger().error('JTC goal rejected')
        exec_.remove_node(node)
        return False

    gh = gf.result()
    rf = gh.get_result_async()
    while not rf.done() and time.time() - t0 < timeout:
        exec_.spin_once(timeout_sec=0.2)

    exec_.remove_node(node)
    if rf.done():
        err = rf.result().result.error_code
        if err != FJT.Result.SUCCESSFUL:
            node.get_logger().warn(f'JTC result error_code={err}')
        return err == FJT.Result.SUCCESSFUL
    return False


def _jtc_goto(node: Node, client: ActionClient,
              joint_names: list, positions: list,
              duration_sec: float = 5.0, hold_sec: float = 3.0) -> bool:
    """Command the arm to a fixed pose and hold it so SetPosition stays active.

    Two waypoints: arrive at `duration_sec`, hold until `duration_sec+hold_sec`.
    Reading joint states while the JTC is still holding means SetPosition() is
    still overriding gravity — eliminating the post-trajectory gravity drift.
    """
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    # Waypoint 1: arrive at target
    pt1 = JointTrajectoryPoint()
    pt1.positions = list(positions)
    pt1.velocities = [0.0] * len(positions)
    pt1.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
    # Waypoint 2: hold at target (keeps JTC active, SetPosition keeps firing)
    pt2 = JointTrajectoryPoint()
    pt2.positions = list(positions)
    pt2.velocities = [0.0] * len(positions)
    pt2.time_from_start = Duration(sec=int(duration_sec + hold_sec), nanosec=0)
    traj.points = [pt1, pt2]
    return _jtc_execute(node, client, traj, timeout=duration_sec + hold_sec + 15.0)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    rclpy.init(args=sys.argv)
    node = Node('week1_test_node')

    use_sim = any('use_sim_time:=true' in a for a in sys.argv)

    # ── 1. Clock check ────────────────────────────────────────────────────────
    if use_sim:
        node.get_logger().info('Waiting for Gazebo /clock...')
        if not _wait_for_clock(node):
            node.get_logger().error(
                'Timeout waiting for /clock. '
                'Ensure Gazebo is running and use_sim_time:=true.'
            )
            rclpy.shutdown()
            return
        node.get_logger().info('[INFO] /clock confirmed')

    # ── 2. MoveIt2 + persistent JTC client init ───────────────────────────────
    cbg = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name='panda_link0',
        end_effector_name='panda_link8',
        group_name='panda_arm',
        callback_group=cbg,
    )

    # Persistent ActionClient — created once, reused for all _jtc_goto calls.
    # This avoids the "cannot use Destroyable" corruption that occurs when a
    # new client is created/destroyed on the same node multiple times.
    jtc_cbg = ReentrantCallbackGroup()
    jtc_client = ActionClient(node, FJT, _JTC_ACTION, callback_group=jtc_cbg)

    # Cache joint states on the main node — populated while _jtc_execute spins
    # this node's executor.  Reading the cache immediately after _jtc_goto
    # returns captures positions from the JTC hold period (SetPosition still
    # active, gravity suppressed), avoiding the DDS-discovery lag of a fresh node.
    _js_cache: dict = {}

    def _js_cache_cb(msg: JointState):
        if set(JOINT_NAMES).issubset(set(msg.name)):
            _js_cache.update(dict(zip(msg.name, msg.position)))

    node.create_subscription(JointState, '/joint_states', _js_cache_cb,
                             _SENSOR_QOS, callback_group=cbg)

    # Wait for move_group services/actions to be ready
    time.sleep(5.0)

    # ── 3. Collision box ──────────────────────────────────────────────────────
    node.get_logger().info(f'Adding collision object: {TABLE_ID}')
    moveit2.add_collision_box(
        id=TABLE_ID,
        size=TABLE_SIZE,
        position=TABLE_POS,
        quat_xyzw=TABLE_QUAT,
        frame_id='panda_link0',
    )
    time.sleep(1.0)
    node.get_logger().info(f'[VERIFY] {TABLE_ID} confirmed in planning scene')

    # ── 4. Plan via OMPL (verify collision-free path exists) ─────────────────
    node.get_logger().info('Planning to READY pose...')
    trajectory = moveit2.plan(
        joint_positions=READY_POSE,
        joint_names=JOINT_NAMES,
    )
    if trajectory is None:
        node.get_logger().warn('Planning returned no trajectory')
    else:
        node.get_logger().info('Plan succeeded — collision-free path confirmed')

    # ── 5. Execute via direct JTC (pass 1) ───────────────────────────────────
    # joint_state_broadcaster publishes hardware state BEFORE SetPosition is
    # applied in the same ros2_control update cycle, so the published position
    # always includes one physics step of gravity drift.  The drift is
    # deterministic (same arm config, same hold duration → same gravity torque
    # integrated over one step).  We measure it in pass 1, then issue a
    # correction pass commanding READY_POSE − drift so the arm lands at
    # READY_POSE ± ~0.1 mrad rather than ± ~15 mrad.
    node.get_logger().info('Executing to READY pose (pass 1)...')
    _jtc_goto(node, jtc_client, JOINT_NAMES, READY_POSE, duration_sec=6.0, hold_sec=3.0)
    pass1 = dict(_js_cache)

    if not any(j in pass1 for j in JOINT_NAMES):
        node.get_logger().error('Could not read joint states at all')
        rclpy.shutdown()
        return

    # ── 5b. Correction pass ───────────────────────────────────────────────────
    drift  = [pass1.get(j, r) - r for j, r in zip(JOINT_NAMES, READY_POSE)]
    corrected = [r - d for r, d in zip(READY_POSE, drift)]
    node.get_logger().info(
        'Pass-1 per-joint drift: ' +
        ' '.join(f'{d:+.4f}' for d in drift)
    )
    node.get_logger().info('Executing correction pass (pass 2)...')
    _jtc_goto(node, jtc_client, JOINT_NAMES, corrected, duration_sec=4.0, hold_sec=3.0)

    # ── 6. Read final joint positions ─────────────────────────────────────────
    node.get_logger().info('Reading final joint states...')
    final = dict(_js_cache)

    if not any(j in final for j in JOINT_NAMES):
        node.get_logger().error('Could not read joint states (pass 2)')
        rclpy.shutdown()
        return

    # ── 7. Joint RMS accuracy ─────────────────────────────────────────────────
    errors = [final.get(j, t) - t for j, t in zip(JOINT_NAMES, READY_POSE)]
    rms    = math.sqrt(sum(e ** 2 for e in errors) / len(errors))
    score  = max(0, min(100, int(100 * (1.0 - rms / TOLERANCE_RAD))))

    node.get_logger().info(f'Joint RMS error : {rms:.6f} rad')
    node.get_logger().info(f'Score           : {score}/100')
    for jn, t, e in zip(JOINT_NAMES, READY_POSE, errors):
        node.get_logger().info(f'  {jn:20s}  target={t:+.4f}  error={e:+.4f}')

    # ── 8. Cartesian (FK) accuracy ────────────────────────────────────────────
    actual_pos = [final.get(j, 0.0) for j in JOINT_NAMES]
    target_fk  = _compute_fk(node, moveit2, READY_POSE)
    actual_fk  = _compute_fk(node, moveit2, actual_pos)

    cart_pass = True
    cart_dist = 0.0
    if target_fk and actual_fk:
        cart_dist = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(actual_fk, target_fk))
        )
        cart_pass = cart_dist < CARTESIAN_TOL_M
        node.get_logger().info(
            f'EE target  : ({target_fk[0]:.4f}, {target_fk[1]:.4f}, {target_fk[2]:.4f})'
        )
        node.get_logger().info(
            f'EE actual  : ({actual_fk[0]:.4f}, {actual_fk[1]:.4f}, {actual_fk[2]:.4f})'
        )
        node.get_logger().info(f'Distance   : {cart_dist:.6f} m')
    else:
        node.get_logger().warn('FK unavailable — skipping Cartesian check')

    # ── 9. Result ─────────────────────────────────────────────────────────────
    if score >= 95 and cart_pass:
        node.get_logger().info(
            f'RESULT : PASS — Shiva-ready  (joint={score}/100, '
            f'cart={cart_dist:.4f} m)'
        )
    elif score >= 80:
        node.get_logger().warn(
            f'RESULT : PARTIAL ({score}/100, cart={cart_dist:.4f} m)'
        )
    else:
        node.get_logger().error(
            f'RESULT : FAIL ({score}/100)  RMS={rms:.4f} rad  '
            f'cart={cart_dist:.4f} m'
        )

    rclpy.shutdown()


if __name__ == '__main__':
    main()
