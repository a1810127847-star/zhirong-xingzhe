import argparse

from stable_baselines3.common.env_checker import check_env

from .ros_gazebo_env import ZhirongGazeboEnv


def parse_args():
    parser = argparse.ArgumentParser(description="Validate the PPO Gazebo environment.")
    parser.add_argument("--steps", type=int, default=12)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--curriculum",
        choices=["straight", "fan", "empty", "single"],
        default="empty",
    )
    parser.add_argument("--action-duration", type=float, default=0.12)
    parser.add_argument(
        "--observation-version",
        choices=["v1_bearing_pi", "v2_bearing_half_pi"],
        default="v2_bearing_half_pi",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    env = ZhirongGazeboEnv(
        curriculum=args.curriculum,
        action_duration=args.action_duration,
        observation_version=args.observation_version,
    )
    try:
        check_env(env, warn=True, skip_render_check=True)
        observation, info = env.reset(seed=args.seed)
        total_reward = 0.0
        completed_steps = 0
        for _ in range(args.steps):
            observation, reward, terminated, truncated, info = env.step(
                env.action_space.sample()
            )
            total_reward += float(reward)
            completed_steps += 1
            if terminated or truncated:
                observation, info = env.reset()
        print(f"PPO_OBSERVATION_SHAPE={observation.shape}")
        print(f"PPO_RANDOM_STEPS={completed_steps}")
        print(f"PPO_RANDOM_REWARD={total_reward:.6f}")
        print(f"PPO_LAST_DISTANCE={info['distance_to_goal']:.6f}")
        print("PPO_ENV_CHECK_OK")
    finally:
        env.close()


if __name__ == "__main__":
    main()
