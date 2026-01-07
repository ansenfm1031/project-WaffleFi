from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple, Any, Optional

import numpy as np

import gymnasium as gym
from gymnasium import spaces



@dataclass
class CoverageConfig:
    # 그리드 크기 (H x W)
    height: int = 12
    width: int = 12

    # 장애물 비율 (0~0.3 정도 권장)
    obstacle_ratio: float = 0.12

    # 에피소드 길이 제한
    max_steps: int = 300

    # coverage 목표 (도달하면 종료)
    target_coverage: float = 0.70

    # 보상 스케일
    reward_new_cell: float = 1.0
    reward_revisit: float = 0.0
    step_penalty: float = -0.01
    collision_penalty: float = -5.0
    goal_bonus: float = 3.0

    # 랜덤 시드
    seed: Optional[int] = 0



class CoverageEnv(gym.Env):
    """
    "안 가본 칸 방문" 보상을 주는 커버리지 탐색용 최소 Gym 환경 (ROS 없이 장난감 버전)

    - State/Observation:
        obs = [ agent_x_norm, agent_y_norm, coverage_ratio, 4방향 장애물/벽(0/1), step_frac ]
        => shape (3 + 4 + 1) = (8,)

    - Action (Discrete 4):
        0: Up, 1: Right, 2: Down, 3: Left

    - Transition:
        action에 따라 1칸 이동 시도, 벽/장애물면 충돌

    - Reward:
        새 칸 방문 +1, 재방문 0, 매 스텝 -0.01, 충돌 -5
        target_coverage 달성 시 +goal_bonus 하고 done=True

    - Done:
        충돌 or max_steps or target_coverage 달성
    """

    metadata = {"render_modes": ["human", "ansi"], "render_fps": 10}

    def __init__(self, config: CoverageConfig = CoverageConfig(), render_mode: Optional[str] = None):
        super().__init__()
        self.cfg = config
        self.render_mode = render_mode

        # RNG
        self.np_random = np.random.default_rng(self.cfg.seed)

        # --- action space ---
        self.action_space = spaces.Discrete(4)

        # --- observation space ---
        # [x_norm, y_norm, coverage, up_block, right_block, down_block, left_block, step_frac]
        low = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)
        high = np.array([1, 1, 1, 1, 1, 1, 1, 1], dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        # 내부 상태들
        self.grid: np.ndarray = None          # 0: free, 1: obstacle
        self.visited: np.ndarray = None       # bool
        self.agent_pos: Tuple[int, int] = (0, 0)
        self.step_count: int = 0
        self.visited_count: int = 0
        self.total_free_cells: int = 1

    # ------------------------
    # Gym API: reset / step
    # ------------------------
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)

        self.step_count = 0

        # 1) 맵 생성(장애물 랜덤)
        self._make_random_grid()

        # 2) 시작 위치(agent) 샘플링 (free cell)
        self.agent_pos = self._sample_free_cell()

        # 3) visited 초기화 + 첫 칸 방문 처리
        self.visited = np.zeros((self.cfg.height, self.cfg.width), dtype=bool)
        self.visited_count = 0
        self._mark_visited(self.agent_pos)

        obs = self._get_obs()
        info = self._get_info()

        # gymnasium은 (obs, info) 리턴, gym은 (obs,)도 허용
        return obs, info

    def step(self, action: int):
        self.step_count += 1

        # 1) action -> 이동 벡터
        dx, dy = self._action_to_delta(action)

        # 2) 다음 위치 계산
        x, y = self.agent_pos
        nx, ny = x + dx, y + dy

        reward = 0.0
        done = False

        # 3) 충돌 체크 (벽/장애물)
        if self._is_blocked(nx, ny):
            reward += self.cfg.collision_penalty
            done = True
            # 충돌이면 위치는 그대로
        else:
            # 이동 성공
            self.agent_pos = (nx, ny)

            # 4) 새 칸 방문 보상
            if not self.visited[nx, ny]:
                self._mark_visited((nx, ny))
                reward += self.cfg.reward_new_cell
            else:
                reward += self.cfg.reward_revisit

        # 5) 시간 패널티(살짝)
        reward += self.cfg.step_penalty

        # 6) 목표 커버리지 달성 여부
        coverage = self._coverage_ratio()
        if coverage >= self.cfg.target_coverage and not done:
            reward += self.cfg.goal_bonus
            done = True

        # 7) 최대 스텝 종료
        if self.step_count >= self.cfg.max_steps:
            done = True

        obs = self._get_obs()
        info = self._get_info()

        # gymnasium 최신 규격은 (obs, reward, terminated, truncated, info)
        # 여기서는 초보용으로 "done 하나"만 쓰되, 호환되게 처리
        terminated = done
        truncated = False  # max_steps를 truncated로 분리하고 싶으면 여기서 처리 가능

        # gym vs gymnasium 호환: gymnasium이면 5개, gym이면 4개
        if "gymnasium" in gym.__name__:
            return obs, float(reward), terminated, truncated, info
        else:
            return obs, float(reward), done, info

    # ------------------------
    # 내부 구현 (환경 구성 요소)
    # ------------------------
    def _make_random_grid(self):
        H, W = self.cfg.height, self.cfg.width
        self.grid = np.zeros((H, W), dtype=np.uint8)

        # 장애물 개수
        num_cells = H * W
        num_obstacles = int(num_cells * self.cfg.obstacle_ratio)

        # 랜덤하게 장애물 배치
        all_indices = [(i, j) for i in range(H) for j in range(W)]
        self.np_random.shuffle(all_indices)

        placed = 0
        for (i, j) in all_indices:
            if placed >= num_obstacles:
                break
            self.grid[i, j] = 1
            placed += 1

        # free cell 수 계산
        self.total_free_cells = int(np.sum(self.grid == 0))
        if self.total_free_cells <= 0:
            # 극단 케이스 방지
            self.grid[:, :] = 0
            self.total_free_cells = H * W

    def _sample_free_cell(self) -> Tuple[int, int]:
        H, W = self.cfg.height, self.cfg.width
        while True:
            i = int(self.np_random.integers(0, H))
            j = int(self.np_random.integers(0, W))
            if self.grid[i, j] == 0:
                return (i, j)

    def _action_to_delta(self, action: int) -> Tuple[int, int]:
        # 0: Up, 1: Right, 2: Down, 3: Left
        if action == 0:
            return (-1, 0)
        if action == 1:
            return (0, 1)
        if action == 2:
            return (1, 0)
        if action == 3:
            return (0, -1)
        raise ValueError(f"Invalid action: {action}")

    def _is_blocked(self, x: int, y: int) -> bool:
        H, W = self.cfg.height, self.cfg.width
        # 벽
        if x < 0 or x >= H or y < 0 or y >= W:
            return True
        # 장애물
        return self.grid[x, y] == 1

    def _mark_visited(self, pos: Tuple[int, int]):
        x, y = pos
        if not self.visited[x, y]:
            self.visited[x, y] = True
            self.visited_count += 1

    def _coverage_ratio(self) -> float:
        return float(self.visited_count) / float(max(1, self.total_free_cells))

    def _get_obs(self) -> np.ndarray:
        x, y = self.agent_pos
        H, W = self.cfg.height, self.cfg.width

        x_norm = x / float(max(1, H - 1))
        y_norm = y / float(max(1, W - 1))
        coverage = self._coverage_ratio()

        # 4방향 blocked 여부
        up = 1.0 if self._is_blocked(x - 1, y) else 0.0
        right = 1.0 if self._is_blocked(x, y + 1) else 0.0
        down = 1.0 if self._is_blocked(x + 1, y) else 0.0
        left = 1.0 if self._is_blocked(x, y - 1) else 0.0

        step_frac = self.step_count / float(max(1, self.cfg.max_steps))

        obs = np.array(
            [x_norm, y_norm, coverage, up, right, down, left, step_frac],
            dtype=np.float32
        )
        return obs

    def _get_info(self) -> Dict[str, Any]:
        return {
            "step": self.step_count,
            "pos": self.agent_pos,
            "coverage_ratio": self._coverage_ratio(),
            "visited_count": int(self.visited_count),
            "free_cells": int(self.total_free_cells),
        }

    # ------------------------
    # Render (옵션)
    # ------------------------
    def render(self):
        if self.render_mode not in ("human", "ansi"):
            return

        H, W = self.cfg.height, self.cfg.width
        ax, ay = self.agent_pos

        # 문자로 맵 출력
        lines = []
        for i in range(H):
            row = []
            for j in range(W):
                if (i, j) == (ax, ay):
                    row.append("A")      # Agent
                elif self.grid[i, j] == 1:
                    row.append("#")      # Obstacle
                elif self.visited is not None and self.visited[i, j]:
                    row.append(".")      # Visited
                else:
                    row.append(" ")      # Unvisited
            lines.append("|" + "".join(row) + "|")

        out = "\n".join(lines)
        if self.render_mode == "human":
            print(out)
        else:
            return out