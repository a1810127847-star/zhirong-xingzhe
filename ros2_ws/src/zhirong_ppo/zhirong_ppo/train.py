import argparse
import datetime as dt
import os
import platform
from pathlib import Path

import gymnasium
import stable_baselines3
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from .ros_gazebo_env import ZhirongGazeboEnv
from .runner import run_episodes, write_json


def parse_args():
    parser = argparse.ArgumentParser(description="Train PPO in the Zhirong Gazebo world.")
    parser.add_argument("--total-timesteps", type=int, default=2048)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--curriculum",
        choices=["straight", "fan", "empty", "single"],
        default="empty",
    )
    parser.add_argument("--action-duration", type=float, default=0.12)
    parser.add_argument("--max-steps", type=int, default=140)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--ent-coef", type=float, default=0.01)
    parser.add_argument(
        "--observation-version",
        choices=["v1_bearing_pi", "v2_bearing_half_pi"],
        default="v2_bearing_half_pi",
    )
    parser.add_argument("--eval-episodes", type=int, default=3)
    parser.add_argument(
        "--checkpoint-freq",
        type=int,
        default=256,
        help="Save a recovery checkpoint every N environment steps; 0 disables it.",
    )
    parser.add_argument("--artifact-root", type=Path, default=Path("artifacts/ppo"))
    parser.add_argument("--run-name", default="")
    parser.add_argument(
        "--resume-model",
        type=Path,
        default=None,
        help="Continue from an existing PPO .zip model.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.n_steps < args.batch_size:
        raise ValueError("--n-steps must be at least --batch-size")
    if args.n_steps % args.batch_size != 0:
        raise ValueError("--n-steps must be divisible by --batch-size")
    if args.checkpoint_freq < 0:
        raise ValueError("--checkpoint-freq must be non-negative")

    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = args.run_name or f"{timestamp}_{args.curriculum}_seed{args.seed}"
    run_dir = args.artifact_root.expanduser().resolve() / run_name
    run_dir.mkdir(parents=True, exist_ok=False)

    resume_model = None
    if args.resume_model is not None:
        resume_model = args.resume_model.expanduser().resolve()
        if not resume_model.is_file():
            raise FileNotFoundError(f"Resume model does not exist: {resume_model}")

    config = {
        "run_name": run_name,
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "total_timesteps": args.total_timesteps,
        "seed": args.seed,
        "curriculum": args.curriculum,
        "action_duration": args.action_duration,
        "max_steps": args.max_steps,
        "n_steps": args.n_steps,
        "batch_size": args.batch_size,
        "learning_rate": args.learning_rate,
        "ent_coef": args.ent_coef,
        "eval_episodes": args.eval_episodes,
        "checkpoint_freq": args.checkpoint_freq,
        "python": platform.python_version(),
        "torch": torch.__version__,
        "gymnasium": gymnasium.__version__,
        "stable_baselines3": stable_baselines3.__version__,
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "action_topic": "/cmd_vel",
        "safe_action_topic": "/cmd_vel_safe",
        "reward_version": "v4_turn_before_drive",
        "observation_version": args.observation_version,
        "resume_model": str(resume_model) if resume_model is not None else None,
    }

    raw_env = ZhirongGazeboEnv(
        curriculum=args.curriculum,
        action_duration=args.action_duration,
        max_steps=args.max_steps,
        observation_version=args.observation_version,
    )
    env = Monitor(raw_env, str(run_dir / "monitor.csv"), info_keywords=(
        "success",
        "collision",
        "path_length",
        "min_scan",
        "safety_interventions",
        "smoothness",
        "max_stall_steps",
        "goal_y",
    ))
    try:
        if resume_model is None:
            model = PPO(
                "MlpPolicy",
                env,
                learning_rate=args.learning_rate,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                gamma=0.99,
                gae_lambda=0.95,
                ent_coef=args.ent_coef,
                verbose=1,
                seed=args.seed,
                device="cpu",
            )
        else:
            model = PPO.load(
                resume_model,
                env=env,
                device="cpu",
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                learning_rate=args.learning_rate,
                ent_coef=args.ent_coef,
                seed=args.seed,
                force_reset=True,
            )
            model.verbose = 1
        config["starting_timesteps"] = int(model.num_timesteps)
        write_json(run_dir / "config.json", config)
        model.set_logger(configure(str(run_dir), ["stdout", "csv"]))
        checkpoint_callback = None
        if args.checkpoint_freq > 0:
            checkpoint_dir = run_dir / "checkpoints"
            checkpoint_dir.mkdir(parents=True, exist_ok=True)
            checkpoint_callback = CheckpointCallback(
                save_freq=args.checkpoint_freq,
                save_path=str(checkpoint_dir),
                name_prefix="ppo_zhirong",
                save_replay_buffer=False,
                save_vecnormalize=False,
                verbose=1,
            )
        model.learn(
            total_timesteps=args.total_timesteps,
            reset_num_timesteps=resume_model is None,
            progress_bar=False,
            callback=checkpoint_callback,
        )
        model_path = run_dir / "ppo_zhirong"
        model.save(model_path)
        evaluation = run_episodes(
            model,
            raw_env,
            args.eval_episodes,
            args.seed + 10000,
        )
        # Keep artifact metadata portable across Windows/WSL path encodings.
        evaluation["model"] = model_path.with_suffix(".zip").name
        write_json(run_dir / "evaluation.json", evaluation)
        print(f"PPO_RUN_DIR={run_dir}")
        print(f"PPO_MODEL={model_path.with_suffix('.zip')}")
        print(f"PPO_TOTAL_TIMESTEPS={model.num_timesteps}")
        print(f"PPO_EVAL_SUCCESS_RATE={evaluation['success_rate']:.6f}")
        print(f"PPO_EVAL_COLLISION_RATE={evaluation['collision_rate']:.6f}")
        print("PPO_TRAINING_OK")
    finally:
        env.close()


if __name__ == "__main__":
    main()
