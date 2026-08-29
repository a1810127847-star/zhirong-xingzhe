import json
from pathlib import Path

import numpy as np


def run_episodes(model, env, episodes, seed):
    results = []
    for episode_index in range(episodes):
        observation, _ = env.reset(seed=seed + episode_index)
        terminated = False
        truncated = False
        episode_reward = 0.0
        last_info = {}
        while not terminated and not truncated:
            action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, last_info = env.step(action)
            episode_reward += float(reward)
        result = {
            key: value
            for key, value in last_info.items()
            if key != "reward_terms"
        }
        result["episode"] = episode_index
        result["reward"] = episode_reward
        results.append(result)

    summary = {
        "episodes": episodes,
        "success_rate": float(np.mean([item["success"] for item in results])),
        "collision_rate": float(np.mean([item["collision"] for item in results])),
        "mean_reward": float(np.mean([item["reward"] for item in results])),
        "mean_path_length": float(
            np.mean([item["path_length"] for item in results])
        ),
        "mean_min_scan": float(np.mean([item["min_scan"] for item in results])),
        "mean_safety_interventions": float(
            np.mean([item["safety_interventions"] for item in results])
        ),
        "mean_max_stall_steps": float(
            np.mean([item["max_stall_steps"] for item in results])
        ),
        "results": results,
    }
    return summary


def write_json(path, payload):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
