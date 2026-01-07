import time
from proj_wifi_heatmap_rl.env.coverage_ros_env import RosCoverageEnv, RosCoverageConfig

def main():
    cfg = RosCoverageConfig(
        step_dt=0.2,
        target_visited_cells=120,   # 처음엔 낮게
        max_steps=400
    )
    env = RosCoverageEnv(cfg)

    obs, info = env.reset()
    print("reset:", info, "obs_dim:", obs.shape)

    total_reward = 0.0
    done = False

    while not done:
        action = env.action_space.sample()  # 랜덤
        out = env.step(action)

        # gymnasium / gym 호환
        if len(out) == 5:
            obs, reward, terminated, truncated, info = out
            done = terminated or truncated
        else:
            obs, reward, done, info = out

        total_reward += reward

        print(
            f"step={info['step']:4d} action={info['last_action']} "
            f"reward={reward:6.3f} visited={info['visited_cells']:4d} "
            f"cov_like={info['coverage_like']:.3f}  x={info['x']:+.2f} y={info['y']:+.2f}"
        )

        # 너무 빠르면 로그만 넘쳐서 약간 쉬어도 됨
        time.sleep(0.02)

    print("DONE total_reward =", total_reward)
    env.close()

if __name__ == "__main__":
    main()
