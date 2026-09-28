"""関節指令の補間区間・速度制限の共通処理。"""
import math

# 始終点の速度・加速度ゼロの5次補間における最大速度係数
quintic_peak_velocity_factor = 1.875


def quintic_max_step(max_joint_velocity, duration_sec):
    """静止端点間の5次補間に対する関節変位上限。"""
    return max_joint_velocity * duration_sec / quintic_peak_velocity_factor


def quintic_duration(current, target, min_duration_sec, max_joint_velocity):
    """関節速度制限を考慮した静止端点間の補間時間。"""
    if len(current) != len(target) or not len(current):
        raise ValueError('関節配列は同じ長さの非空配列が必要です')
    if not all(math.isfinite(value) for value in [*current, *target]):
        raise ValueError('関節角は有限値が必要です')
    if not all(math.isfinite(value) and value > 0 for value in (min_duration_sec, max_joint_velocity)):
        raise ValueError('時間と速度制限は正の有限値が必要です')
    max_delta = max(abs(a-b) for a, b in zip(target, current))
    return max(min_duration_sec, quintic_peak_velocity_factor * max_delta / max_joint_velocity)


def make_rest_to_rest_trajectory(joint_names, current, target, duration_sec):
    """始終点の速度・加速度ゼロのJointTrajectory。実補間はコントローラ側。"""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    current, target = list(current), list(target)
    if not len(joint_names) or len(joint_names) != len(current) or len(current) != len(target):
        raise ValueError('関節名と始終点は同じ長さの非空配列が必要です')
    if not math.isfinite(duration_sec) or duration_sec <= 0:
        raise ValueError('補間時間は正の有限値が必要です')
    if not all(math.isfinite(value) for value in current + target):
        raise ValueError('関節角は有限値が必要です')
    message = JointTrajectory()
    message.joint_names = list(joint_names)
    initial = JointTrajectoryPoint(positions=current, velocities=[0.0]*len(current), accelerations=[0.0]*len(current))
    final = JointTrajectoryPoint(positions=target, velocities=[0.0]*len(target), accelerations=[0.0]*len(target))
    final.time_from_start.sec = int(duration_sec)
    final.time_from_start.nanosec = int((duration_sec-int(duration_sec))*1e9)
    message.points = [initial, final]
    return message
