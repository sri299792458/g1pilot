from copy import deepcopy


G1_NUM_MOTORS = 35

LEG_MOTOR_IDS = tuple(range(12))
WAIST_YAW_MOTOR_ID = 12
WAIST_ROLL_MOTOR_ID = 13
WAIST_PITCH_MOTOR_ID = 14
WAIST_MOTOR_IDS = (WAIST_YAW_MOTOR_ID, WAIST_ROLL_MOTOR_ID, WAIST_PITCH_MOTOR_ID)
LEFT_ARM_MOTOR_IDS = tuple(range(15, 22))
RIGHT_ARM_MOTOR_IDS = tuple(range(22, 29))
ARM_MOTOR_IDS = LEFT_ARM_MOTOR_IDS + RIGHT_ARM_MOTOR_IDS
UPPER_BODY_MOTOR_IDS = WAIST_MOTOR_IDS + ARM_MOTOR_IDS
LOCKED_WAIST_MOTOR_IDS = (WAIST_ROLL_MOTOR_ID, WAIST_PITCH_MOTOR_ID)

# OpenHomie observes the 27 active joints: 12 legs, waist yaw, and 14 arms.
OPENHOMIE_OBS_MOTOR_IDS = LEG_MOTOR_IDS + (WAIST_YAW_MOTOR_ID,) + ARM_MOTOR_IDS


def parse_int_list(value, default=()):
    if value is None:
        return tuple(default)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return tuple(default)
        return tuple(int(item.strip()) for item in text.split(",") if item.strip())
    return tuple(int(item) for item in value)


def parse_float_list(value, default=()):
    if value is None:
        return tuple(default)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return tuple(default)
        return tuple(float(item.strip()) for item in text.split(",") if item.strip())
    return tuple(float(item) for item in value)


def motor_state_count(lowstate):
    try:
        return len(lowstate.motor_state)
    except Exception:
        return 0


def motor_q(lowstate, motor_id, default=0.0):
    if lowstate is None or motor_id >= motor_state_count(lowstate):
        return float(default)
    return float(lowstate.motor_state[motor_id].q)


def motor_dq(lowstate, motor_id, default=0.0):
    if lowstate is None or motor_id >= motor_state_count(lowstate):
        return float(default)
    return float(lowstate.motor_state[motor_id].dq)


def set_motor_command(cmd, motor_id, *, mode, q, kp, kd, dq=0.0, tau=0.0):
    motor_cmd = cmd.motor_cmd[motor_id]
    motor_cmd.mode = int(mode)
    motor_cmd.q = float(q)
    motor_cmd.kp = float(kp)
    motor_cmd.dq = float(dq)
    motor_cmd.kd = float(kd)
    motor_cmd.tau = float(tau)


def set_passive_motor_command(cmd, lowstate, motor_id):
    set_motor_command(
        cmd,
        motor_id,
        mode=0,
        q=motor_q(lowstate, motor_id),
        kp=0.0,
        kd=0.0,
    )


def copy_motor_command(dst_cmd, src_cmd, motor_id):
    dst_cmd.motor_cmd[motor_id] = deepcopy(src_cmd.motor_cmd[motor_id])


def copy_command_metadata(dst_cmd, lowstate=None, src_cmd=None):
    if src_cmd is not None:
        dst_cmd.mode_pr = int(getattr(src_cmd, "mode_pr", 0))
        dst_cmd.mode_machine = int(getattr(src_cmd, "mode_machine", 0))
    if lowstate is not None:
        dst_cmd.mode_machine = int(getattr(lowstate, "mode_machine", dst_cmd.mode_machine))


def clamp(value, lower, upper):
    return max(float(lower), min(float(upper), float(value)))
