import argparse
from pathlib import Path

from stable_baselines3 import PPO

from .ros_gazebo_env import ZhirongGazeboEnv
from .runner import run_episodes, write_json


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate a saved Zhirong PPO model.")
    parser.add_argument("model", type=Path)
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--curriculum",
        choices=["straight", "fan", "empty", "single"],
        default="empty",
    )
    parser.add_argument("--action-duration", type=float, default=0.12)
    parser.add_argument("--max-steps", type=int, default=140)
    parser.add_argument(
        "--observation-version",
        choices=["v1_bearing_pi", "v2_bearing_half_pi"],
        default="v2_bearing_half_pi",
    )
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    model = PPO.load(args.model, device="cpu")
    env = ZhirongGazeboEnv(
        curriculum=args.curriculum,
        action_duration=args.action_duration,
        max_steps=args.max_steps,
        observation_version=args.observation_version,
    )
    try:
        evaluation = run_episodes(model, env, args.episodes, args.seed)
        output = args.output or args.model.with_name("evaluation_30_seed.json")
        write_json(output, evaluation)
        print(f"PPO_EVAL_OUTPUT={output.resolve()}")
        print(f"PPO_EVAL_SUCCESS_RATE={evaluation['success_rate']:.6f}")
        print(f"PPO_EVAL_COLLISION_RATE={evaluation['collision_rate']:.6f}")
        print("PPO_EVALUATION_OK")
    finally:
        env.close()


if __name__ == "__main__":
    main()
