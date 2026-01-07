#!/usr/bin/env python3
import time
import argparse
import numpy as np
import torch
import torch.nn as nn

# 네 환경 import 경로에 맞게 수정
from proj_wifi_heatmap_rl.env.coverage_ros_env import RosCoverageEnv


# -----------------------------
# Q Network (train과 동일해야 함)
# -----------------------------
class QNet(nn.Module):
    def __init__(self, obs_dim: int, n_actions: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, n_actions),
        )

    def forward(self, x):
        return self.net(x)


@torch.no_grad()
def select_action_greedy(qnet, obs, device):
    x = torch.from_numpy(obs).float().to(device).unsqueeze(0)
    q = qnet(x)
    return int(torch.argmax(q, dim=1).item())


def recovery(env: RosCoverageEnv):
    # 1) stop 0.2s
    env._hold_action(0.0, 0.0, 0.2)

    # 2) back 0.6s (env에 후진이 없으면 직접 publish로 구현 필요)
    env._hold_action(-0.10, 0.0, 0.6)

    # 후진이 env에 없다면 rotate만으로도 복구 가능:
    # 2) rotate 0.9s
    env._hold_action(0.0, +1.2, 0.9)

    # 3) forward 0.4s
    env._hold_action(+0.12, 0.0, 0.4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=str, required=True, help="path to trained .pt")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--sleep", type=float, default=0.05, help="step delay (sec)")
    parser.add_argument("--done", action="store_true")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"

    # -----------------------------
    # Env 생성
    # -----------------------------
    env = RosCoverageEnv()
    if hasattr(env, "cfg") and hasattr(env.cfg, "max_steps"):
        env.cfg.max_steps = 20000
    if hasattr(env, "cfg") and hasattr(env.cfg, "target_visited_cells"):
        env.cfg.target_visited_cells = 150

    # obs / action dim 추론
    obs = env.reset()
    if isinstance(obs, tuple):
        obs = obs[0]
    obs = np.asarray(obs, dtype=np.float32)

    obs_dim = obs.shape[0]
    # n_actions = 4  # 네 action 정의 기준
    # n_actions = 6
    n_actions = int(env.action_space.n) if hasattr(env, "action_space") else 6

    # -----------------------------
    # Model 로드
    # -----------------------------
    qnet = QNet(obs_dim, n_actions).to(device)

    ckpt = torch.load(args.ckpt, map_location=device)
    qnet.load_state_dict(ckpt["qnet"])
    qnet.eval()

    print(f"[LOAD] {args.ckpt}")
    print(f"[INFO] obs_dim={obs_dim}, n_actions={n_actions}, device={device}")

    # -----------------------------
    # Run episodes
    # -----------------------------
    for ep in range(1, args.episodes + 1):
        obs = env.reset()
        if isinstance(obs, tuple):
            obs = obs[0]
        obs = np.asarray(obs, dtype=np.float32)

        ep_reward = 0.0
        step = 0

        while True:
            action = select_action_greedy(qnet, obs, device)

            out = env.step(action)
            if len(out) == 5:
                next_obs, reward, terminated, truncated, info = out
                done = terminated or truncated
            else:
                next_obs, reward, done, info = out

            obs = np.asarray(next_obs, dtype=np.float32)
            ep_reward += float(reward)
            step += 1

            if args.sleep > 0:
                time.sleep(args.sleep)

            if done:
                print(f"[DONE] step: {info.get('step', 0)} visited_cells: {info.get('visited_cells', 0)} collision: {info.get('collision', 0)}")
                if args.done:
                    break
                else:
                    print("[RECOVERY]")
                    recovery(env)

        print(f"[EP {ep}] steps={step} reward={ep_reward:.2f}")

    env.close()
    print("[DONE]")


if __name__ == "__main__":
    main()
