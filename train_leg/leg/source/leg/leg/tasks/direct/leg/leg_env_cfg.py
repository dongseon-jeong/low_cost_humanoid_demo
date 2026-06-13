# leg_env_cfg.py

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .legs_cfg import LEGS_CFG

from isaaclab.sensors import ContactSensorCfg

@configclass
class LegEnvCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 10.0

    action_space = 12
    observation_space = 48
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1.0 / 120.0, render_interval=decimation)

    robot_cfg: ArticulationCfg = LEGS_CFG.replace(
        prim_path="/World/envs/env_.*/legs"   # / 로 시작
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1024,
        env_spacing=2.0,
        replicate_physics=True,
    )

    joint_names = [
        "lbase_joint",
        "rbase_joint",
        "ll1_joint",
        "rl1_joint",
        "ll2_joint",
        "rl2_joint",
        "ll3_joint",
        "rl3_joint",
        "ll4_joint",
        "rl4_joint",
        "ll5_joint",
        "rl5_joint",
    ]

    action_scale = 0.3
    torque_limit = 8.0

    # 베이스 높이
    base_height_target = 0.2
    min_base_height = 0.10
    max_base_pitch = 1.0
    max_base_roll = 1.0

    # reward scales
    rew_scale_alive = 0.5      # 1.0 너무 크지 않게, 그래도 살아있으면 + 보상
    rew_scale_terminated = -5.0     # 넘어지면 꽤 큰 음수
    rew_scale_upright = 3.0         # 3 자세 잘 유지하면 꽤 보상
    
    rew_scale_forward_vel = 25.0     # 앞으로 가면 많이 보상
    rew_vel_track_rate = 1.0 # 5 속도 증가

    # penalties는 일단 아주 약하게 시작
    rew_scale_joint_vel = -1e-4
    rew_scale_action_rate = -1e-4
    rew_scale_energy = -0.0005          # 처음엔 꺼버려도 됨
    rew_heading_rate = 1.0   # 5~20

    # 양발 접촉 방지
    rew_scale_leg_interference = -0.005   # 마이너스 보상(패널티)
    leg_interference_force_threshold = 100.0  # N 단위(대충 시작값)

    # ---- step / alternation / support-time shaping ----
    alternation_rew_rate = 10.0        # 교대 성공 보상
    same_foot_pen_rate = 0.0         # 같은 발 연속 landing 패널티
    rew_lat_pen_rate = 0 # -2.0
    rew_yaw_pen_rate = 0 # -0.5 

    # 스텝용

    foot_target_width = 0.06

    # ---- foot contact thresholds (hysteresis) ----
    foot_th_update_interval = 200     # steps
    foot_th_q_on = 0.70               # on = Q70
    foot_th_q_off = 0.30 # 0.40              # off = Q40
    foot_th_ema = 0.90                # EMA smoothing (0.9~0.99 추천)
    foot_th_min_on = 600.0
    foot_th_max_on = 6000.0
    foot_th_min_off = 200.0 # 300.0
    foot_th_max_off = 5000.0
    foot_th_margin = 100.0 # 200.0            # ensure on >= off + margin

    # fallback 초기값(학습 초반에 quantile이 0일 수도 있어서)
    foot_contact_force_th_on_init  = 1400.0 # 2200.0 
    foot_contact_force_th_off_init = 1000.0 # 2000.0

    # ---- Foot contact: ONLY ground/terrain ----
    GROUND_FILTER = ["/World/GroundPlane/GroundPlane/CollisionPlane"]

    scene.left_foot_contact = ContactSensorCfg(
        prim_path="/World/envs/env_.*/legs/ll6_.*",
        filter_prim_paths_expr=GROUND_FILTER,   # 추가
        update_period=0.0,
        history_length=1,
    )
    scene.right_foot_contact = ContactSensorCfg(
        prim_path="/World/envs/env_.*/legs/rl6_.*",
        filter_prim_paths_expr=GROUND_FILTER,   # 추가
        update_period=0.0,
        history_length=1,
    )



    # ---- 발 간섭 ----
    scene.left_leg_contact = ContactSensorCfg(
        prim_path="/World/envs/env_.*/legs/ll(4|5|6)_.*",
        filter_prim_paths_expr=["/World/envs/env_.*/legs/rl(4|5|6)_.*"],
        update_period=0.0,
        history_length=1,
    )
    scene.right_leg_contact = ContactSensorCfg(
        prim_path="/World/envs/env_.*/legs/rl(4|5|6)_.*",
        filter_prim_paths_expr=["/World/envs/env_.*/legs/ll(4|5|6)_.*"], #ll.*
        update_period=0.0,
        history_length=1,
    )
