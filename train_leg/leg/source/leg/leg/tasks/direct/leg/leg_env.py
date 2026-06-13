# leg_env.py
# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from collections.abc import Sequence

import torch
import os
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sensors import ContactSensor  # <-- make sure your IsaacLab version provides this

from .leg_env_cfg import LegEnvCfg


class LegEnv(DirectRLEnv):
    cfg: LegEnvCfg

    def __init__(self, cfg: LegEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # --- Contact sensors (must exist in cfg/scene) ---
        # Foot contact (ground reaction)
        self._left_foot_contact: ContactSensor = self.scene.sensors["left_foot_contact"]
        self._right_foot_contact: ContactSensor = self.scene.sensors["right_foot_contact"]

        # Leg-leg interference (optional but used in reward)
        self._left_leg_interf: ContactSensor = self.scene.sensors["left_leg_contact"]
        self._right_leg_interf: ContactSensor = self.scene.sensors["right_leg_contact"]

        # DOF indices for controlled joints
        self._dof_indices, _ = self.robot.find_joints(self.cfg.joint_names)
        self._num_dofs = len(self.cfg.joint_names)

        # State views (updated each step)
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel

        # Action buffers
        self.actions = torch.zeros(self.num_envs, self._num_dofs, device=self.device)
        self.last_actions = torch.zeros_like(self.actions)

        # Torque proxy (we use position targets; keep zeros unless you add explicit torque control)
        self.joint_torques = torch.zeros_like(self.actions)

        # Termination debounce counters
        self._low_count = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
        self._tilt_count = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)

        # Contact hysteresis state
        self.last_c_l = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.last_c_r = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)

        # Landing alternation tracking
        self.last_landing_valid = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.last_landing_is_left = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)

        # Per-foot thresholds (python floats; safe for TorchScript comparisons)
        self.th_on_l = float(self.cfg.foot_contact_force_th_on_init)
        self.th_off_l = float(self.cfg.foot_contact_force_th_off_init)
        self.th_on_r = float(self.cfg.foot_contact_force_th_on_init)
        self.th_off_r = float(self.cfg.foot_contact_force_th_off_init)

        self.freeze_foot_thresholds = False          # True면 업데이트 안 함
        self.enable_foot_th_update = True            # 학습에서 True, 추론에서 False로 둘 수 있음

        # foot body ids (exact names)
        lf_ids, _ = self.robot.find_bodies("ll6_1")
        rf_ids, _ = self.robot.find_bodies("rl6_1")

        self._lf_body_id = int(lf_ids[0])
        self._rf_body_id = int(rf_ids[0])

        self.prev_base_pos = torch.zeros((self.num_envs, 3), device=self.device)

    # ---------------------------------------------------------------------
    # Scene
    # ---------------------------------------------------------------------
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/GroundPlane", cfg=GroundPlaneCfg())

        self.scene.clone_environments(copy_from_source=False)

        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self.scene.articulations["robot"] = self.robot

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    # ---------------------------------------------------------------------
    # RL hooks
    # ---------------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        # Keep last actions for action-rate penalty
        self.last_actions[:] = self.actions

        # Smooth + clamp incoming actions
        alpha = float(self.cfg.action_smoothing_alpha) if hasattr(self.cfg, "action_smoothing_alpha") else 0.2
        actions = torch.clamp(actions, -1.0, 1.0)
        self.actions[:] = (1.0 - alpha) * self.actions + alpha * actions

    def _apply_action(self) -> None:
        # Clamp final action
        act = torch.clamp(self.actions, -1.0, 1.0)

        # Ramp-in at episode start (avoid immediate explosions)
        ramp_seconds = float(self.cfg.action_ramp_seconds) if hasattr(self.cfg, "action_ramp_seconds") else 0.5
        ramp_steps = int(ramp_seconds / (self.cfg.sim.dt * self.cfg.decimation))
        ramp = torch.clamp(
            self.episode_length_buf.float() / float(max(ramp_steps, 1)),
            0.0,
            1.0,
        ).unsqueeze(-1)  # [N,1]

        # Default pose for controlled joints
        q0 = self.robot.data.default_joint_pos[:, self._dof_indices]  # [N,D]

        # action_scale is "radian delta"
        q_target = q0 + (act * float(self.cfg.action_scale)) * ramp

        # Position targets (implicit PD from actuators)
        self.robot.set_joint_position_target(q_target, joint_ids=self._dof_indices)

        # Torque proxy (kept at zero unless you compute/measure real torques)
        self.joint_torques.zero_()

    def _get_observations(self) -> dict:
        # Refresh joint states
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel

        root_state = self.robot.data.root_state_w
        base_pos = root_state[:, 0:3]
        base_quat = root_state[:, 3:7]     # expected (qw,qx,qy,qz)
        base_lin_vel = root_state[:, 7:10]
        base_ang_vel = root_state[:, 10:13]

        # Prefer COM height if available
        if hasattr(self.robot.data, "root_com_pos_w"):
            base_height = self.robot.data.root_com_pos_w[:, 2]
        elif hasattr(self.robot.data, "com_pos_w"):
            base_height = self.robot.data.com_pos_w[:, 2]
        else:
            base_height = base_pos[:, 2]

        tilt_angle = compute_tilt_from_quat_wxyz(base_quat)

        # Controlled joints
        q = self.joint_pos[:, self._dof_indices]
        qd = self.joint_vel[:, self._dof_indices]

        # Action history
        last_act = self.last_actions

        # Extra scalars
        forward_vel = base_lin_vel[:, 0].unsqueeze(-1)
        height_err = (base_height - float(self.cfg.base_height_target)).unsqueeze(-1)
        x_pos = base_pos[:, 0].unsqueeze(-1)
        y_pos = base_pos[:, 1].unsqueeze(-1)

        obs = torch.cat(
            (
                q,                       # D
                qd,                      # D
                base_lin_vel,            # 3
                base_ang_vel,            # 3
                base_height.unsqueeze(-1),  # 1
                tilt_angle.unsqueeze(-1),   # 1
                last_act,                # D
                forward_vel,             # 1
                x_pos,                   # 1
                y_pos,                   # 1
                height_err,              # 1
            ),
            dim=-1,
        )

        obs = torch.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        obs = torch.clamp(obs, -100.0, 100.0)

        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        # Refresh states
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel

        root_state = self.robot.data.root_state_w
        base_pos = root_state[:, 0:3]
        base_quat = root_state[:, 3:7]     # expected (qw,qx,qy,qz)
        base_lin_vel = root_state[:, 7:10]
        base_ang_vel = root_state[:, 10:13]

        # Prefer COM height if available
        if hasattr(self.robot.data, "root_com_pos_w"):
            base_height = self.robot.data.root_com_pos_w[:, 2]
        elif hasattr(self.robot.data, "com_pos_w"):
            base_height = self.robot.data.com_pos_w[:, 2]
        else:
            base_height = base_pos[:, 2]

        tilt_angle = compute_tilt_from_quat_wxyz(base_quat)

        qd = self.joint_vel[:, self._dof_indices]
        action_rate = self.actions - self.last_actions

        # -------------------------
        # Leg-leg interference force
        # -------------------------
        leg_interf_max_force = compute_leg_interference_max_force(
            self._left_leg_interf.data.force_matrix_w,
            self._right_leg_interf.data.force_matrix_w,
            self.num_envs,
            self.device,
        )

        # -------------------------
        # Foot contact (Fz) + auto thresholds (hysteresis)
        # -------------------------
        nf_l = self._left_foot_contact.data.net_forces_w
        nf_r = self._right_foot_contact.data.net_forces_w

        # shape 맞춰서 [N,3]로 통일
        if nf_l is not None and nf_l.dim() == 3:
            nf_l0 = nf_l[:, 0, :]
        else:
            nf_l0 = nf_l

        if nf_r is not None and nf_r.dim() == 3:
            nf_r0 = nf_r[:, 0, :]
        else:
            nf_r0 = nf_r

        # # 반드시 nf_l0/nf_r0로 힘 계산 (nf_l/nf_r 그대로 쓰면 shape/축 처리 문제로 이상해질 수 있음)
        # nf_l0, nf_r0: [N,3] (world frame)
        fz_l_raw = nf_l0[:, 2]
        fz_r_raw = nf_r0[:, 2]

        # 어떤 쪽이 "접촉 시 +값"인지 자동으로 맞추기 (한 번에 결정)
        # 지금 로그만 보면 L은 +가 접촉, R은 -가 접촉처럼 보임.
        # => R은 부호 뒤집어야 함.
        fz_l = torch.clamp(fz_l_raw, min=0.0)
        fz_r = torch.clamp(-fz_r_raw, min=0.0)   # 여기 중요



        # ------------------------------------------------------------------
        # [추가] threshold 업데이트는 "접촉 후보 샘플"만 사용 (비접촉 이상힘 제거)
        # ------------------------------------------------------------------
        if (not self.freeze_foot_thresholds) and self.enable_foot_th_update:
            # 1) 우선은 last contact 기반으로 필터링 (가장 안전/간단)
            #    - 이미 접촉 중인 env들의 힘 분포로만 quantile을 뽑는다
            mask_l = self.last_c_l
            mask_r = self.last_c_r

            # 2) 접촉 샘플이 너무 적으면 업데이트 스킵하거나,
            #    pre-contact 후보(작은 임계)로 보조 마스크를 만든다
            min_samples = int(getattr(self.cfg, "foot_th_min_samples", 64))

            # (옵션) pre-contact 임계(너무 낮게 잡으면 노이즈 들어옴. 보통 50~200 사이)
            pre_th = float(getattr(self.cfg, "foot_th_pre_contact", 150.0))
            if mask_l.sum().item() < min_samples:
                mask_l = mask_l | (fz_l > pre_th)
            if mask_r.sum().item() < min_samples:
                mask_r = mask_r | (fz_r > pre_th)

            # 그래도 샘플 부족하면 그냥 이번 업데이트는 건너뛰기
            if (mask_l.sum().item() >= min_samples) and (mask_r.sum().item() >= min_samples):
                fz_l_u = fz_l[mask_l]
                fz_r_u = fz_r[mask_r]

                self._update_thresholds_quantile(fz_l_u, fz_r_u)

        # # contact force 계산 뒤
        # if (not self.freeze_foot_thresholds) and self.enable_foot_th_update:
        #     self._update_thresholds_quantile(fz_l, fz_r)

        c_l = contact_bool_hysteresis_from_fz(fz_l, self.last_c_l, self.th_on_l, self.th_off_l)
        c_r = contact_bool_hysteresis_from_fz(fz_r, self.last_c_r, self.th_on_r, self.th_off_r)

        # --- Foot flatness shaping (only when in contact) ---
        bs = self.robot.data.body_state_w

        lf_quat_xyzw = bs[:, self._lf_body_id, 3:7]
        rf_quat_xyzw = bs[:, self._rf_body_id, 3:7]

        lf_cos = foot_flatness_cos_from_quat_xyzw(lf_quat_xyzw)  # [-1..1]
        rf_cos = foot_flatness_cos_from_quat_xyzw(rf_quat_xyzw)

        # "평평"은 cos가 1에 가까울수록 좋음. 음수는 0으로 클램프해서 upside-down 영향 제거
        lf_cos = torch.clamp(lf_cos, 0.0, 1.0)
        rf_cos = torch.clamp(rf_cos, 0.0, 1.0)

        landing_l = c_l & (~self.last_c_l)
        landing_r = c_r & (~self.last_c_r)
        self.last_c_l[:] = c_l
        self.last_c_r[:] = c_r
        flight = (~c_l) & (~c_r)

        # -------------------------
        # Alternation (landing-based)
        # -------------------------
        any_landing  = landing_l | landing_r
        both_landing = landing_l & landing_r

        # prev state (from last step)
        prev_left  = self.last_landing_is_left        # bool tensor [N]
        prev_valid = self.last_landing_valid          # bool tensor [N]

        # 교대 성공/실패 판정 (이번 스텝 landing 이벤트에 대해서만)
        alt_good = prev_valid & (~both_landing) & (
            (landing_l & (~prev_left)) | (landing_r & prev_left)
        )
        alt_bad = prev_valid & (~both_landing) & (
            (landing_l & prev_left) | (landing_r & (~prev_left))
        )

        # Update last landing foot (ignore simultaneous landing)
        single_landing = any_landing & (~both_landing)

        new_left = torch.where(single_landing & landing_l, torch.ones_like(prev_left), prev_left)
        new_left = torch.where(single_landing & landing_r, torch.zeros_like(prev_left), new_left)

        self.last_landing_is_left[:] = new_left
        self.last_landing_valid[:] = prev_valid | single_landing

        # 다리사이 간격

        lf_pos = bs[:, self._lf_body_id, 0:3]
        rf_pos = bs[:, self._rf_body_id, 0:3]





        # -------------------------
        # Reward
        # -------------------------
        total_reward = compute_rewards(
            # scales
            float(self.cfg.rew_scale_alive),
            float(self.cfg.rew_scale_terminated),
            float(self.cfg.rew_scale_forward_vel),
            float(self.cfg.rew_scale_upright),
            float(self.cfg.rew_scale_joint_vel),
            float(self.cfg.rew_scale_action_rate),
            float(self.cfg.rew_scale_energy),
            float(self.cfg.rew_scale_leg_interference),
            float(self.cfg.leg_interference_force_threshold),
            # shaping rates
            float(self.cfg.rew_vel_track_rate),
            float(self.cfg.alternation_rew_rate),
            float(self.cfg.same_foot_pen_rate),
            float(self.cfg.rew_lat_pen_rate),
            float(self.cfg.rew_yaw_pen_rate),
            leg_interf_max_force,
            landing_l,
            landing_r,
            alt_good,
            alt_bad,
            # robot state tensors
            base_quat,
            base_ang_vel,
            base_lin_vel,
            base_height,
            tilt_angle,
            qd,
            self.joint_torques,
            action_rate,
            float(self.cfg.base_height_target),
            self.reset_terminated,
            float(self.cfg.rew_heading_rate),
            lf_pos,
            rf_pos,
            float(self.cfg.foot_target_width),

        )

        if int(self.common_step_counter) > 99998 == 0: 
            self.save_foot_thresholds("logs/foot_th.pt")

        if int(self.common_step_counter) % 500 == 0:
            with torch.no_grad():
                v_fwd_dbg = compute_forward_velocity(base_quat, base_lin_vel, forward_sign=-1.0)
                flight_ratio = flight.float().mean().item()

                import omni.usd
                bad = ((~c_l) & (fz_l > 200.0)) | ((~c_r) & (fz_r > 200.0))
                print("[DBG] bad_noncontact_force cnt:", bad.sum().item())
                if not hasattr(self, "_printed_ground_paths"):
                    self._printed_ground_paths = True
                    stage = omni.usd.get_context().get_stage()
                    hits = []
                    for prim in stage.Traverse():
                        p = prim.GetPath().pathString
                        if "GroundPlane" in p and ("Collision" in p or "collision" in p):
                            hits.append(p)
                    print("[DBG] GroundPlane prim hits (first 50):")
                    for p in hits[:50]:
                        print("  ", p)
                    print("[DBG] total hits:", len(hits), flush=True)

                self._dbg_print_contact_sensor_fields()
                n = fz_l_raw.numel()
                neg_l = (fz_l_raw < 0).float().mean().item()
                pos_l = (fz_l_raw > 0).float().mean().item()
                neg_r = (fz_r_raw < 0).float().mean().item()
                pos_r = (fz_r_raw > 0).float().mean().item()

                # 몇 개 분위수도 같이 보면 더 확실
                l_q10 = float(torch.quantile(fz_l_raw, 0.10).item())
                l_q90 = float(torch.quantile(fz_l_raw, 0.90).item())
                r_q10 = float(torch.quantile(fz_r_raw, 0.10).item())
                r_q90 = float(torch.quantile(fz_r_raw, 0.90).item())

                print(f"[DBG fz_raw sign] L neg/pos={neg_l:.2f}/{pos_l:.2f} q10/q90={l_q10:.1f}/{l_q90:.1f} | "
                    f"R neg/pos={neg_r:.2f}/{pos_r:.2f} q10/q90={r_q10:.1f}/{r_q90:.1f}")

                # 1) 전체 통계
                print("alt :", alt_good.sum().item())
                print("[DBG] v_fwd mean/max:", v_fwd_dbg.mean().item(), v_fwd_dbg.max().item(), flush=True)
                print("[DBG] contact ratio L/R:", c_l.float().mean().item(), c_r.float().mean().item(), flush=True)
                print("[DBG] flight ratio:", flight_ratio, flush=True)
                print("[DBG] th L on/off:", self.th_on_l, self.th_off_l, " | R:", self.th_on_r, self.th_off_r, flush=True)
                print("[DBG] flat cos L/R mean:", lf_cos.mean().item(), rf_cos.mean().item(), flush=True)

                # 2) 힘 raw shape 확인 (L/R이 같은 걸 읽는지, shape 꼬였는지 바로 보임)
                def _shape(x):
                    return None if x is None else tuple(x.shape)
                print("[DBG] nf_l shape:", _shape(nf_l), "nf_l0 shape:", _shape(nf_l0), flush=True)
                print("[DBG] nf_r shape:", _shape(nf_r), "nf_r0 shape:", _shape(nf_r0), flush=True)

                # 3) contact / non-contact 분리 평균 (여기서 swing에서 0 근처가 아니면 '가공/오프셋' 가능성 큼)
                eps = 1e-6
                n_cl = c_l.float().sum().clamp_min(1.0)
                n_cr = c_r.float().sum().clamp_min(1.0)
                n_sl = (~c_l).float().sum().clamp_min(1.0)
                n_sr = (~c_r).float().sum().clamp_min(1.0)

                mean_l_stance = (fz_l * c_l.float()).sum() / n_cl
                mean_r_stance = (fz_r * c_r.float()).sum() / n_cr
                mean_l_swing  = (fz_l * (~c_l).float()).sum() / n_sl
                mean_r_swing  = (fz_r * (~c_r).float()).sum() / n_sr

                print("[DBG |F|] L min/mean/max:", fz_l.min().item(), fz_l.mean().item(), fz_l.max().item(), flush=True)
                print("[DBG |F|] R min/mean/max:", fz_r.min().item(), fz_r.mean().item(), fz_r.max().item(), flush=True)
                print("[DBG |F|] stance mean L/R:", mean_l_stance.item(), mean_r_stance.item(), flush=True)
                print("[DBG |F|] swing  mean L/R:", mean_l_swing.item(), mean_r_swing.item(), flush=True)

                # 4) L/R 차이 분포 (진짜로 비슷한지 수치로 확정)
                diff = (fz_l - fz_r)
                print("[DBG |F|] (L-R) min/mean/max:",
                    diff.min().item(), diff.mean().item(), diff.max().item(), flush=True)

                # 5) 샘플 env 하나만 찍기 (센서 매핑 꼬임이면 여기서 바로 느낌 옴)
                idx = 0
                l_vec = nf_l0[idx].tolist() if nf_l0 is not None else None
                r_vec = nf_r0[idx].tolist() if nf_r0 is not None else None
                print(f"[DBG sample env0] cL/cR={bool(c_l[idx].item())}/{bool(c_r[idx].item())} "
                    f"fzL/fzR={fz_l[idx].item():.3f}/{fz_r[idx].item():.3f} "
                    f"nfL0={l_vec} nfR0={r_vec}", flush=True)

        
        return total_reward

    def _get_dones(self):
        root = self.robot.data.root_state_w
        pos = root[:, 0:3]
        linvel = root[:, 7:10]
        angvel = root[:, 10:13]
        base_quat = root[:, 3:7]  # expected (qw,qx,qy,qz)

        # Prefer COM height if available
        if hasattr(self.robot.data, "root_com_pos_w"):
            base_height = self.robot.data.root_com_pos_w[:, 2]
        elif hasattr(self.robot.data, "com_pos_w"):
            base_height = self.robot.data.com_pos_w[:, 2]
        else:
            base_height = pos[:, 2]

        tilt_angle = compute_tilt_from_quat_wxyz(base_quat)

        max_tilt = float(max(self.cfg.max_base_pitch, self.cfg.max_base_roll))

        # Safety
        bad_nan = torch.isnan(root).any(dim=1) | torch.isinf(root).any(dim=1)
        bad_oob = (pos.abs().max(dim=1).values > 100.0)
        bad_vel = (linvel.abs().max(dim=1).values > 20.0) | (angvel.abs().max(dim=1).values > 50.0)
        bad = bad_nan | bad_oob | bad_vel

        # Raw conditions
        low = base_height < float(self.cfg.min_base_height)
        tilt = tilt_angle > max_tilt

        # Debounce
        self._low_count = torch.where(low, self._low_count + 1, torch.zeros_like(self._low_count))
        self._tilt_count = torch.where(tilt, self._tilt_count + 1, torch.zeros_like(self._tilt_count))

        low_term_steps = int(getattr(self.cfg, "low_term_steps", 12))
        tilt_term_steps = int(getattr(self.cfg, "tilt_term_steps", 8))

        low_term = self._low_count >= low_term_steps
        tilt_term = self._tilt_count >= tilt_term_steps

        fallen = low_term | tilt_term | bad

        time_out = self.episode_length_buf >= self.max_episode_length - 1
        time_out = time_out & ~fallen

        return fallen, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        if not isinstance(env_ids, torch.Tensor):
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        # Reset joints
        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]

        # Reset root over env origins
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        # Buffers
        self.actions[env_ids].zero_()
        self.last_actions[env_ids].zero_()
        self.joint_torques[env_ids].zero_()

        self._low_count[env_ids] = 0
        self._tilt_count[env_ids] = 0

        self.last_c_l[env_ids] = False
        self.last_c_r[env_ids] = False

        self.last_landing_valid[env_ids] = False
        self.last_landing_is_left[env_ids] = False

        # Optionally reset thresholds to init per episode
        if bool(getattr(self.cfg, "reset_contact_thresholds_on_reset", False)):
            self.th_on_l = float(self.cfg.foot_contact_force_th_on_init)
            self.th_off_l = float(self.cfg.foot_contact_force_th_off_init)
            self.th_on_r = float(self.cfg.foot_contact_force_th_on_init)
            self.th_off_r = float(self.cfg.foot_contact_force_th_off_init)

        # Write to sim
        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids=env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids=env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

        self.prev_base_pos[env_ids] = self.robot.data.root_state_w[env_ids, 0:3]

    # ---------------------------------------------------------------------
    # Threshold auto-update (single definition; duplicates removed)
    # ---------------------------------------------------------------------
    def _update_thresholds_quantile(self, fz_l: torch.Tensor, fz_r: torch.Tensor) -> None:
        # cfg에 foot_th_update_interval 등이 있을 때만 동작
        interval = int(getattr(self.cfg, "foot_th_update_interval", 0))
        if interval <= 0:
            return

        if int(self.common_step_counter) % interval != 0:
            return

        q_on = float(getattr(self.cfg, "foot_th_q_on", 0.70))
        q_off = float(getattr(self.cfg, "foot_th_q_off", 0.40))

        # quantile
        on_l_t = torch.quantile(fz_l, q_on)
        off_l_t = torch.quantile(fz_l, q_off)
        on_r_t = torch.quantile(fz_r, q_on)
        off_r_t = torch.quantile(fz_r, q_off)

        # clamp
        on_l = float(torch.clamp(on_l_t,  min=float(self.cfg.foot_th_min_on),  max=float(self.cfg.foot_th_max_on)).item())
        off_l = float(torch.clamp(off_l_t, min=float(self.cfg.foot_th_min_off), max=float(self.cfg.foot_th_max_off)).item())
        on_r = float(torch.clamp(on_r_t,  min=float(self.cfg.foot_th_min_on),  max=float(self.cfg.foot_th_max_on)).item())
        off_r = float(torch.clamp(off_r_t, min=float(self.cfg.foot_th_min_off), max=float(self.cfg.foot_th_max_off)).item())

        # hysteresis margin
        m = float(getattr(self.cfg, "foot_th_margin", 200.0))
        if on_l < off_l + m:
            on_l = off_l + m
        if on_r < off_r + m:
            on_r = off_r + m

        # EMA
        a = float(getattr(self.cfg, "foot_th_ema", 0.90))
        self.th_on_l  = a * self.th_on_l  + (1.0 - a) * on_l
        self.th_off_l = a * self.th_off_l + (1.0 - a) * off_l
        self.th_on_r  = a * self.th_on_r  + (1.0 - a) * on_r
        self.th_off_r = a * self.th_off_r + (1.0 - a) * off_r

    def _th_state_dict(self):
        # float tensor/number 모두 안전하게 저장
        return {
            "th_on_l":  torch.as_tensor(self.th_on_l).detach().cpu(),
            "th_off_l": torch.as_tensor(self.th_off_l).detach().cpu(),
            "th_on_r":  torch.as_tensor(self.th_on_r).detach().cpu(),
            "th_off_r": torch.as_tensor(self.th_off_r).detach().cpu(),
        }

    def save_foot_thresholds(self, path: str):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save(self._th_state_dict(), path)

    def load_foot_thresholds(self, path: str, strict: bool = True):
        obj = torch.load(path, map_location="cpu")

        # scalar/tensor 모두 대응
        def _to_device(x):
            t = torch.as_tensor(x, device=self.device, dtype=torch.float32)
            return t

        try:
            self.th_on_l  = _to_device(obj["th_on_l"]).item() if not torch.is_tensor(self.th_on_l) else _to_device(obj["th_on_l"])
            self.th_off_l = _to_device(obj["th_off_l"]).item() if not torch.is_tensor(self.th_off_l) else _to_device(obj["th_off_l"])
            self.th_on_r  = _to_device(obj["th_on_r"]).item() if not torch.is_tensor(self.th_on_r) else _to_device(obj["th_on_r"])
            self.th_off_r = _to_device(obj["th_off_r"]).item() if not torch.is_tensor(self.th_off_r) else _to_device(obj["th_off_r"])
        except Exception as e:
            if strict:
                raise
            print(f"[WARN] failed to load thresholds from {path}: {e}")

        # 추론에서는 업데이트 끄는 게 보통 정답
        self.freeze_foot_thresholds = True

    def _dbg_print_contact_sensor_fields(self):
        # 너무 자주 찍히면 로그 폭발하니 interval로 제한
        interval = int(getattr(self.cfg, "dbg_contact_fields_interval", 5000))
        if interval <= 0:
            return
        if int(self.common_step_counter) % interval != 0:
            return

        for name, sensor in [("L", self._left_foot_contact), ("R", self._right_foot_contact)]:
            d = sensor.data
            # data 객체에 어떤 attribute가 있는지 key만 추려서 출력
            keys = []
            for k in dir(d):
                if k.startswith("_"):
                    continue
                try:
                    v = getattr(d, k)
                except Exception:
                    continue
                # 텐서/숫자/리스트류만
                if torch.is_tensor(v) or isinstance(v, (int, float, list, tuple)):
                    keys.append(k)
            print(f"[ContactSensor {name}] available data fields: {sorted(keys)[:50]}{' ...' if len(keys)>50 else ''}")

            # 자주 쓰는 후보 몇 개는 값/shape도 같이 찍기 (있을 때만)
            for k in ["net_forces_w", "forces_w", "normals_w", "contact_count", "contact_counts",
                    "contact_forces_w", "contact_force_matrix_w", "contact_body_ids", "contact_actor_ids"]:
                if hasattr(d, k):
                    v = getattr(d, k)
                    if torch.is_tensor(v):
                        print(f"[ContactSensor {name}] {k}: shape={tuple(v.shape)} dtype={v.dtype} device={v.device}")
                    else:
                        print(f"[ContactSensor {name}] {k}: {type(v)}")


# =============================================================================
# Helpers (TorchScript-safe where it matters)
# =============================================================================

@torch.jit.script
def quat_apply_wxyz(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    # q: [N,4] = (qw,qx,qy,qz), v: [N,3]
    qw = q[:, 0]
    qx = q[:, 1]
    qy = q[:, 2]
    qz = q[:, 3]

    vx = v[:, 0]
    vy = v[:, 1]
    vz = v[:, 2]

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)

    return torch.stack((vpx, vpy, vpz), dim=1)


@torch.jit.script
def compute_tilt_from_quat_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    # quat_wxyz: [N,4] = (qw, qx, qy, qz) -> tilt angle [0..pi]
    qw = quat_wxyz[:, 0]
    qx = quat_wxyz[:, 1]
    qy = quat_wxyz[:, 2]
    qz = quat_wxyz[:, 3]

    # R33 = 1 - 2*(x^2 + y^2) for quaternion (w,x,y,z)
    z_wz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return torch.acos(torch.clamp(z_wz, -1.0, 1.0))


@torch.jit.script
def foot_force_mag_safe(nf: torch.Tensor) -> torch.Tensor:
    if nf.numel() == 0:
        return torch.zeros((0,), device=nf.device)

    if nf.dim() == 2 and nf.size(1) == 3:
        return torch.linalg.norm(nf, dim=1)

    if nf.dim() == 3 and nf.size(-1) == 3:
        # [N,K,3] -> sum of magnitudes over contacts
        return torch.linalg.norm(nf, dim=-1).sum(dim=1)

    # fallback
    nf2 = nf.reshape(nf.size(0), -1)
    return torch.abs(nf2).sum(dim=1)


@torch.jit.script
def contact_bool_hysteresis_from_fz(
    fz: torch.Tensor,           # [N]
    prev_contact: torch.Tensor, # bool [N]
    th_on: float,
    th_off: float,
) -> torch.Tensor:
    fz_ = fz.view(-1)
    prev_ = prev_contact.view(-1)

    turn_on = fz_ > th_on
    turn_off = fz_ < th_off
    return torch.where(prev_, ~turn_off, turn_on)

def compute_leg_interference_max_force(
    fm_l: torch.Tensor | None,
    fm_r: torch.Tensor | None,
    num_envs: int,
    device,
) -> torch.Tensor:
    # force_matrix_w is typically [N, B, C, 3] or [N, 1, C, 3] etc.
    def max_mag(fm: torch.Tensor | None) -> torch.Tensor:
        if fm is None:
            return torch.zeros((num_envs,), device=device)
        mag = torch.linalg.norm(fm, dim=-1)  # [..., 3] -> [...]
        # reduce all dims except env dim
        while mag.dim() > 1:
            mag = mag.max(dim=-1).values
        return mag

    max_l = max_mag(fm_l)
    max_r = max_mag(fm_r)
    return torch.maximum(max_l, max_r)

@torch.jit.script
def compute_forward_velocity(base_quat: torch.Tensor, base_lin_vel: torch.Tensor, forward_sign: float) -> torch.Tensor:
    N = base_quat.shape[0]
    axis_x = base_quat.new_zeros((N, 3))
    axis_x[:, 0] = 1.0
    fwd_w = quat_apply_wxyz(base_quat, axis_x)
    fwd_true = forward_sign * fwd_w
    return torch.sum(base_lin_vel * fwd_true, dim=1)

@torch.jit.script
def quat_xyzw_to_wxyz(q_xyzw: torch.Tensor) -> torch.Tensor:
    # [N,4] (qx,qy,qz,qw) -> (qw,qx,qy,qz)
    return torch.stack((q_xyzw[:, 3], q_xyzw[:, 0], q_xyzw[:, 1], q_xyzw[:, 2]), dim=1)

@torch.jit.script
def foot_flatness_cos_from_quat_xyzw(foot_quat_xyzw: torch.Tensor) -> torch.Tensor:
    # 로컬 up(0,0,1)을 월드로 회전시킨 뒤, 월드 up과 내적 = cos
    q_wxyz = quat_xyzw_to_wxyz(foot_quat_xyzw)
    N = q_wxyz.shape[0]
    up_local = q_wxyz.new_zeros((N, 3))
    up_local[:, 2] = 1.0
    up_world = quat_apply_wxyz(q_wxyz, up_local)  # [N,3]
    # world up = (0,0,1) 이므로 dot은 z성분
    return up_world[:, 2]  # [-1..1]

@torch.jit.script
def compute_rewards(
    # scales
    rew_scale_alive: float,
    rew_scale_terminated: float,
    rew_scale_forward_vel: float,
    rew_scale_upright: float,
    rew_scale_joint_vel: float,
    rew_scale_action_rate: float,
    rew_scale_energy: float,
    rew_scale_leg_interference: float,
    leg_interference_force_threshold: float,
    # shaping rates
    rew_vel_track_rate: float,
    alternation_rew_rate: float,
    same_foot_pen_rate: float,
    rew_lat_pen_rate: float,
    rew_yaw_pen_rate: float,
    # tensors (events/states)
    leg_interf_max_force: torch.Tensor,  # [N]
    landing_l: torch.Tensor,             # bool [N]
    landing_r: torch.Tensor,             # bool [N]
    alt_good: torch.Tensor,             # bool [N]
    alt_bad: torch.Tensor,              # bool [N]
    # robot state
    base_quat: torch.Tensor,            # [N,4] (qw,qx,qy,qz)
    base_ang_vel: torch.Tensor,         # [N,3]
    base_lin_vel: torch.Tensor,         # [N,3]
    base_height: torch.Tensor,          # [N]
    tilt_angle: torch.Tensor,           # [N]
    joint_vel: torch.Tensor,            # [N,D]
    joint_torques: torch.Tensor,        # [N,D]
    action_rate: torch.Tensor,          # [N,D]
    base_height_target: float,
    reset_terminated: torch.Tensor,     # [N] bool/int
    rew_heading_rate:float,

    lf_pos: torch.Tensor,
    rf_pos: torch.Tensor,
    foot_target_width:float,


) -> torch.Tensor:

    # Alive / termination
    alive_term = (1.0 - reset_terminated.float())
    rew_alive = rew_scale_alive * alive_term
    rew_termination = rew_scale_terminated * reset_terminated.float()

    # Robot frame axes in world
    N = base_quat.shape[0]
    axis_x = base_quat.new_zeros((N, 3))
    axis_y = base_quat.new_zeros((N, 3))
    axis_z = base_quat.new_zeros((N, 3))
    axis_x[:, 0] = 1.0
    axis_y[:, 1] = 1.0
    axis_z[:, 2] = 1.0

    fwd_w = quat_apply_wxyz(base_quat, axis_x)
    lat_w = quat_apply_wxyz(base_quat, axis_y)
    up_w = quat_apply_wxyz(base_quat, axis_z)

    # If true forward is -X
    forward_sign = -1.0
    fwd_true = forward_sign * fwd_w

    v_fwd = torch.sum(base_lin_vel * fwd_true, dim=1)
    v_lat = torch.sum(base_lin_vel * lat_w, dim=1)
    yaw_rate = torch.sum(base_ang_vel * up_w, dim=1)

    # Forward reward (clipped)
    rew_forward = rew_scale_forward_vel * torch.clamp(v_fwd, 0.0, 2.0)

    # Velocity tracking (gaussian peak at v_cmd, baseline-shifted to ~0 at v=0)
    v_cmd = 0.3# 0.8
    k = 0.5 # 2.0
    baseline_in = base_quat.new_full((1,), -k * v_cmd * v_cmd)   # [1]
    baseline = torch.exp(baseline_in)[0]                         # scalar tensor
    rew_vel_track = rew_vel_track_rate * (torch.exp(-k * (v_fwd - v_cmd) * (v_fwd - v_cmd)) - baseline)

    # # (C) 정지 억제 (v_fwd가 너무 작으면 패널티)
    # v_eps = 0.05
    # stand_pen = torch.relu(v_eps - v_fwd)
    # rew_stand_pen = -2.5 * (stand_pen * stand_pen)

    # # (D) crab-walk / yaw 강 억제
    # rew_lat_pen = rew_lat_pen_rate * (v_lat * v_lat)        # ← 기존 -1.0 보다 강하게
    # rew_yaw_pen = rew_yaw_pen_rate  * (yaw_rate * yaw_rate)  # ← 기존 -0.1 보다 강하게

    # Heading alignment to world -X
    target_dir = base_quat.new_zeros((N, 3))
    target_dir[:, 0] = -1.0

    fwd_xy = fwd_true[:, 0:2]
    tgt_xy = target_dir[:, 0:2]
    fwd_xy = fwd_xy / (torch.linalg.norm(fwd_xy, dim=1, keepdim=True) + 1e-6)
    tgt_xy = tgt_xy / (torch.linalg.norm(tgt_xy, dim=1, keepdim=True) + 1e-6)

    heading_cos = torch.sum(fwd_xy * tgt_xy, dim=1)
    rew_heading = rew_heading_rate * torch.clamp(heading_cos, 0.0, 1.0)

    # Upright / height stabilization
    height_err = base_height - base_height_target
    upright_term = torch.exp(-5.0 * height_err * height_err - 4.0 * tilt_angle * tilt_angle)
    rew_upright = rew_scale_upright * upright_term

    # Small penalties
    rew_joint_vel = rew_scale_joint_vel * torch.sum(joint_vel * joint_vel, dim=1)
    rew_action_rate = rew_scale_action_rate * torch.sum(action_rate * action_rate, dim=1)
    rew_energy = rew_scale_energy * torch.sum(torch.abs(joint_torques * joint_vel), dim=1)

    # Leg interference penalty
    # excess = torch.clamp(leg_interf_max_force - leg_interference_force_threshold, min=0.0)
    # excess = torch.clamp(excess, max=200.0)
    # rew_leg_interf = rew_scale_leg_interference * (excess / 200.0)

    # Alternation shaping
    # rew_alt = alternation_rew_rate * alt_good.to(torch.float32) \
    #         + same_foot_pen_rate * alt_bad.to(torch.float32)

    foot_width = torch.abs(lf_pos[:, 1] - rf_pos[:, 1])

    width_err = foot_width - foot_target_width

    rew_foot_spacing = torch.exp(50.0 * width_err * width_err)

    total = (
        rew_alive
        + rew_termination
        + rew_upright
        + rew_forward
        + rew_vel_track
        + rew_heading
        # + rew_leg_interf
        + rew_joint_vel
        + rew_action_rate
        + rew_energy
        # + rew_alt
        # + rew_lat_pen
        # + rew_yaw_pen
        + rew_foot_spacing
    )
    return total