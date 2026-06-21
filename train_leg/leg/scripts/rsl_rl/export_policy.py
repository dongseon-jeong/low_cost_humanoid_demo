import torch
import os

from isaaclab.app import AppLauncher

# Isaac Sim 초기화
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

BASE_PATH = os.environ.get("BASE_PATH")
import sys
sys.path.append(BASE_PATH+"/leg/source/leg")
import leg.tasks  # noqa: F401


from isaaclab_tasks.utils import parse_env_cfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

from rsl_rl.runners import OnPolicyRunner
from leg.tasks.direct.leg.agents.rsl_rl_ppo_cfg import PPORunnerCfg

task_name = "Template-Leg-Direct-v0"


checkpoint_path = (
    "logs/rsl_rl/leg/"
    "model_9999.pt"
)


export_path = "policy_jit.pt"


# 환경 설정
env_cfg = parse_env_cfg(
    task_name,
    device="cuda:0",
    num_envs=1,
)

import gymnasium as gym


env = gym.make(
    task_name,
    cfg=env_cfg
)

env = RslRlVecEnvWrapper(env)


agent_cfg = PPORunnerCfg()


runner = OnPolicyRunner(
    env,
    agent_cfg.to_dict(),
    log_dir=None,
    device="cuda:0"
)


# checkpoint load
runner.load(checkpoint_path)
print(runner.alg)
print(dir(runner.alg))

# policy 가져오기
policy = runner.alg.policy.actor
policy = policy.to("cpu")
policy.eval()


# observation dimension 확인
obs_dim = env.observation_space.shape[0]


dummy_input = torch.randn(
    (1, 48),
    dtype=torch.float32,
    device="cpu"
)


# TorchScript 변환
traced_policy = torch.jit.trace(
    policy,
    dummy_input
)


traced_policy.save(
    export_path
)


print(
    "Export complete:",
    export_path
)


simulation_app.close()