from proj_wifi_heatmap_rl.env.coverage_env import CoverageEnv, CoverageConfig

def main():
    env = CoverageEnv(CoverageConfig(height=10, width=10, obstacle_ratio=0.10), render_mode="human")

    obs, info = env.reset()
    print("reset obs:", obs)
    print("reset info:", info)
    env.render()

    # 랜덤 행동으로 한 에피소드 돌려보기
    done = False
    total_reward = 0.0

    # gymnasium이면 5개, gym이면 4개라 분기
    while not done:
        action = env.action_space.sample()

        out = env.step(action)
        if len(out) == 5:
            obs, reward, terminated, truncated, info = out
            done = terminated or truncated
        else:
            obs, reward, done, info = out

        total_reward += reward
        print(f"action={action}, reward={reward:.3f}, coverage={info['coverage_ratio']:.3f}")
        env.render()

    print("episode done. total_reward =", total_reward)

if __name__ == "__main__":
    main()
