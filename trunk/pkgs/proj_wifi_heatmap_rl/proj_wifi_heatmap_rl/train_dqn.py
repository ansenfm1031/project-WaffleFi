#!/usr/bin/env python3
import os
import time
import random
from collections import deque
from dataclasses import dataclass

import argparse

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from proj_wifi_heatmap_rl.env.coverage_ros_env import RosCoverageEnv


# -----------------------------
# Replay Buffer
# -----------------------------
class ReplayBuffer:
    def __init__(self, capacity: int, obs_dim: int):
        self.capacity = capacity
        self.obs_dim = obs_dim
        self.buf = deque(maxlen=capacity)

    def push(self, s, a, r, s2, done):
        self.buf.append((s, a, r, s2, done))

    def sample(self, batch_size: int):
        batch = random.sample(self.buf, batch_size)
        s, a, r, s2, done = zip(*batch)
        return (
            np.stack(s).astype(np.float32),
            np.array(a, dtype=np.int64),
            np.array(r, dtype=np.float32),
            np.stack(s2).astype(np.float32),
            np.array(done, dtype=np.float32),
        )

    def __len__(self):
        return len(self.buf)


# -----------------------------
# Q Network
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


# -----------------------------
# Hyperparams
# -----------------------------
@dataclass
class HParams:
    seed: int = 0
    total_episodes: int = 800

    gamma: float = 0.99
    # lr: float = 1e-3
    lr: float = 3e-4

    replay_size: int = 100_000
    batch_size: int = 128
    learning_starts: int = 2000          # 리플레이가 이만큼 쌓이기 전엔 학습 X
    # train_every: int = 1                 # 매 스텝마다 학습(느리면 2~4)
    train_every: int = 2
    # target_update_every: int = 1000      # 스텝 단위 하드 업데이트
    target_update_every: int = 2000

    eps_start: float = 1.0
    eps_end: float = 0.05
    # eps_decay_steps: int = 80_000        # 스텝 기준으로 선형 감쇠
    eps_decay_steps: int = 150_000

    grad_clip: float = 10.0
    save_every_episodes: int = 50
    best_window: int = 50
    best_cells_max_collision_rate: float = 0.60  # 최근 window 내 충돌 비율 상한
    best_cells_min_avg_steps: int = 150          # 최근 window 내 평균 steps 하한(선택)
    out_dir: str = "runs_dqn"


def set_seed(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def linear_eps(step, eps_start, eps_end, decay_steps):
    if step >= decay_steps:
        return eps_end
    t = step / decay_steps
    return eps_start + t * (eps_end - eps_start)


@torch.no_grad()
def select_action(qnet: QNet, obs: np.ndarray, n_actions: int, eps: float, device: str):
    if random.random() < eps:
        return random.randrange(n_actions)
    x = torch.from_numpy(obs).float().to(device).unsqueeze(0)  # [1, obs_dim]
    q = qnet(x)  # [1, n_actions]
    return int(torch.argmax(q, dim=1).item())


def train_step(qnet, target, optim_, batch, gamma, grad_clip, device):
    # DQN
    # s, a, r, s2, done = batch
    # s = torch.from_numpy(s).to(device)
    # a = torch.from_numpy(a).to(device)
    # r = torch.from_numpy(r).to(device)
    # s2 = torch.from_numpy(s2).to(device)
    # done = torch.from_numpy(done).to(device)

    # # Q(s,a)
    # q = qnet(s)  # [B, A]
    # q_sa = q.gather(1, a.view(-1, 1)).squeeze(1)  # [B]

    # # target: r + gamma * max_a' Q_target(s',a') * (1-done)
    # with torch.no_grad():
    #     q2 = target(s2)
    #     max_q2 = torch.max(q2, dim=1).values
    #     y = r + gamma * max_q2 * (1.0 - done)

    # loss = nn.SmoothL1Loss()(q_sa, y)

    # optim_.zero_grad(set_to_none=True)
    # loss.backward()
    # nn.utils.clip_grad_norm_(qnet.parameters(), grad_clip)
    # optim_.step()

    # return float(loss.item())

    # Double DQN (DDQN)
    s, a, r, s2, done = batch
    s = torch.from_numpy(s).to(device)
    a = torch.from_numpy(a).to(device)
    r = torch.from_numpy(r).to(device)
    s2 = torch.from_numpy(s2).to(device)
    done = torch.from_numpy(done).to(device)

    # Q(s,a)
    q = qnet(s)  # [B, A]
    q_sa = q.gather(1, a.view(-1, 1)).squeeze(1)  # [B]

    with torch.no_grad():
        # ✅ Double DQN:
        # 1) online 네트워크로 다음 상태에서 argmax action 선택
        next_actions = torch.argmax(qnet(s2), dim=1)  # [B]

        # 2) target 네트워크로 그 action의 Q값 평가
        q2_target = target(s2)  # [B, A]
        q2_sa = q2_target.gather(1, next_actions.view(-1, 1)).squeeze(1)  # [B]

        y = r + gamma * q2_sa * (1.0 - done)

    loss = nn.SmoothL1Loss()(q_sa, y)

    optim_.zero_grad(set_to_none=True)
    loss.backward()
    nn.utils.clip_grad_norm_(qnet.parameters(), grad_clip)
    optim_.step()

    return float(loss.item())


def save_ckpt(path: str, ep: int, global_step: int,
              qnet, target, optim_,
              best_avg_R: float, best_avg_N: float, hp):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    torch.save(
        {
            "episode": ep,
            "global_step": global_step,
            "qnet": qnet.state_dict(),
            "target": target.state_dict(),
            "optim": optim_.state_dict(),
            "best_average_R": float(best_avg_R),
            "best_average_N": float(best_avg_N),
            "hparams": hp.__dict__,
        },
        path,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", type=str, default="", help="checkpoint .pt path")
    parser.add_argument("--more-episodes", type=int, default=0,
                    help="number of additional episodes to train after resume")
    parser.add_argument("--total-episodes", type=int, default=0,
                    help="override total episodes (absolute)")
    parser.add_argument("--reset-optim", action="store_true", help="do not load optimizer state")
    args = parser.parse_args()

    hp = HParams()
    set_seed(hp.seed)

    os.makedirs(hp.out_dir, exist_ok=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # ✅ 네 환경 생성
    env = RosCoverageEnv()

    # obs_dim / action_dim 확인
    # env.observation_space.shape[0], env.action_space.n 형태면 베스트
    try:
        obs_dim = int(env.observation_space.shape[0])
        n_actions = int(env.action_space.n)
    except Exception:
        # 혹시 space가 없으면 reset해보고 추론
        obs = env.reset()
        if isinstance(obs, tuple):  # (obs, info)
            obs = obs[0]
        obs_dim = int(np.asarray(obs).shape[0])
        n_actions = 4  # 네 action=4 고정이면
    print(f"[INFO] obs_dim={obs_dim}, n_actions={n_actions}, device={device}")

    qnet = QNet(obs_dim, n_actions).to(device)
    target = QNet(obs_dim, n_actions).to(device)
    target.load_state_dict(qnet.state_dict())
    target.eval()

    optim_ = optim.Adam(qnet.parameters(), lr=hp.lr)
    rb = ReplayBuffer(hp.replay_size, obs_dim)

    start_episode = 1
    end_episode = hp.total_episodes
    global_step = 0
    resume_step = 0

    recent_rewards = deque(maxlen=hp.best_window)
    best_average_R = -float("inf")
    recent_new_cells = deque(maxlen=hp.best_window)
    best_average_N = -float("inf")
    recent_collisions = deque(maxlen=hp.best_window)
    recent_steps = deque(maxlen=hp.best_window)

    t0 = time.time()

    if args.resume:
        ckpt = torch.load(args.resume, map_location=device)
        qnet.load_state_dict(ckpt["qnet"])
        target.load_state_dict(ckpt.get("target", ckpt["qnet"]))

        if (not args.reset_optim) and ("optim" in ckpt):
            optim_.load_state_dict(ckpt["optim"])

        start_episode = int(ckpt.get("episode", 0)) + 1
        global_step = int(ckpt.get("global_step", 0))
        resume_step = global_step

        best_average_R = ckpt.get("best_average_R", -float("inf"))
        best_average_N = ckpt.get("best_average_N", -float("inf"))

        if args.total_episodes > 0:
            end_episode = args.total_episodes
        elif args.more_episodes > 0:
            end_episode = (start_episode - 1) + args.more_episodes


        print(f"[RESUME] from={args.resume} "
              f"start_episode={start_episode} "
              f"end_episode={end_episode} "
              f"global_step={global_step} "
              f"best_avg_R={best_average_R:.2f} "
              f"best_avg_N={best_average_N:.2f}")

    # for ep in range(1, hp.total_episodes + 1):
    for ep in range(start_episode, end_episode + 1):
        obs = env.reset()
        if isinstance(obs, tuple):
            obs = obs[0]
        obs = np.asarray(obs, dtype=np.float32)

        ep_reward = 0.0
        ep_loss = 0.0
        ep_updates = 0

        for step in range(env.cfg.max_steps):
            eps = linear_eps(global_step, hp.eps_start, hp.eps_end, hp.eps_decay_steps)
            if args.resume and (global_step - resume_step) < 20_000:
                eps = max(eps, 0.2)
            action = select_action(qnet, obs, n_actions, eps, device)

            out = env.step(action)
            if len(out) == 5:
                next_obs, reward, terminated, truncated, info = out
                done = bool(terminated or truncated)
            else:
                # Gym 구버전 호환: (obs, reward, done, info)
                next_obs, reward, done, info = out

            next_obs = np.asarray(next_obs, dtype=np.float32)
            rb.push(obs, action, float(reward), next_obs, float(done))

            obs = next_obs
            ep_reward += float(reward)
            global_step += 1

            # 학습
            if len(rb) >= hp.learning_starts and (global_step % hp.train_every == 0):
                batch = rb.sample(hp.batch_size)
                loss = train_step(qnet, target, optim_, batch, hp.gamma, hp.grad_clip, device)
                ep_loss += loss
                ep_updates += 1

            # 타겟 네트워크 업데이트
            if global_step % hp.target_update_every == 0 and len(rb) >= hp.learning_starts:
                target.load_state_dict(qnet.state_dict())

            if done:
                break
            
            # raw = info.get('min_dist_raw', float("inf"))
            # filt = info.get('min_dist_filt', float("inf"))
            # if raw < 1.0 or filt < 1.0:
            #     print(f"    [DEBUG] min_dist_raw : {raw:.2f} min_dist_filt : {filt:.2f}")

        avg_loss = (ep_loss / max(ep_updates, 1))
        dt = time.time() - t0
        print(
            f"[EP {ep:04d}] R={ep_reward:8.2f} "
            f"steps={step+1:4d} eps={eps:5.3f} "
            f"buf={len(rb):6d} loss={avg_loss:7.4f} "
            f"t={dt:7.1f}s"
        )
        if done:
            print(
                f"    ↳ new_cells={info.get('new_cells_this_ep', 0)} "
                f"collision={info.get('collision_this_ep', False)} "
                f"turn_ratio={info.get('turn_ratio', 0.0):.2f}"
            )

        recent_rewards.append(ep_reward)
        avg_R = np.mean(recent_rewards)
        recent_new_cells.append(info.get('new_cells_this_ep', 0))
        avg_N = np.mean(recent_new_cells)
        recent_collisions.append(1 if info.get('collision_this_ep', False) else 0)
        recent_steps.append(step + 1)  # 이번 에피소드 step 수

        # 주기 저장
        if ep % hp.save_every_episodes == 0:
            ckpt_path = os.path.join(hp.out_dir, f"dqn_ep{ep:04d}.pt")
            save_ckpt(ckpt_path, ep, global_step,
                      qnet, target, optim_,
                      best_average_R, best_average_N, hp)
            print(f"[SAVE] {ckpt_path}")
        
        if len(recent_rewards) == hp.best_window:
            # BEST checkpoint 저장 (R)
            if avg_R > best_average_R:
                best_average_R = avg_R

                best_path = os.path.join(hp.out_dir, "dqn_best_r.pt")
                save_ckpt(best_path, ep, global_step,
                        qnet, target, optim_,
                        best_average_R, best_average_N, hp)
                print(f"[BEST_R] ep={ep} avg_R({hp.best_window})={avg_R:.2f} -> saved dqn_best_r.pt")
        
            # BEST checkpoint 저장 (N)
            col_rate = float(np.mean(recent_collisions))
            avg_steps = float(np.mean(recent_steps))
            if avg_N > best_average_N and col_rate <= hp.best_cells_max_collision_rate and avg_steps >= hp.best_cells_min_avg_steps:
                best_average_N = avg_N

                best_path = os.path.join(hp.out_dir, "dqn_best_n.pt")
                save_ckpt(best_path, ep, global_step,
                        qnet, target, optim_,
                        best_average_R, best_average_N, hp)
                print(f"[BEST_N] ep={ep} avg_N({hp.best_window})={avg_N:.2f} -> saved dqn_best_n.pt")


    env.close()
    print("[DONE]")


if __name__ == "__main__":
    main()
