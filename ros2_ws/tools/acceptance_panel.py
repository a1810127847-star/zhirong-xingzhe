#!/usr/bin/env python3
"""Windows click-to-run acceptance panel for the Zhirong ROS2 simulation."""

from __future__ import annotations

import argparse
import json
import os
import queue
import re
import shlex
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from urllib.parse import urlsplit


ROS_SETUP = "/opt/ros/humble/setup.bash"
TOOLS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TOOLS_DIR.parent.parent
CONFIG_PATH = PROJECT_ROOT / ".acceptance_panel.local.json"
LOG_DIR = PROJECT_ROOT / "artifacts" / "acceptance_panel_logs"
CREATE_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
CONFIG_KEYS = ("wsl_distro", "wsl_workspace", "repo_url", "branch", "source_root")
DEFAULT_CONFIG = {
    "wsl_distro": os.environ.get("ZHIRONG_WSL_DISTRO", "Ubuntu-22.04"),
    "wsl_workspace": os.environ.get(
        "ZHIRONG_WORKSPACE", "$HOME/zhirong_xingzhe_ws"
    ),
    "repo_url": "",
    "branch": "master",
    "source_root": str(PROJECT_ROOT),
}


def load_local_config() -> dict[str, str]:
    config = DEFAULT_CONFIG.copy()
    if CONFIG_PATH.is_file():
        try:
            payload = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                for key in CONFIG_KEYS:
                    value = payload.get(key)
                    if isinstance(value, str) and value.strip():
                        config[key] = value.strip()
        except (OSError, ValueError, TypeError):
            pass
    return config


RUNTIME_CONFIG = load_local_config()


def workspace_shell_ref(suffix: str = "") -> str:
    """Return a quoted Bash workspace path while preserving $HOME expansion."""
    root = RUNTIME_CONFIG["wsl_workspace"].rstrip("/")
    if root == "~":
        root = "$HOME"
    elif root.startswith("~/"):
        root = "$HOME/" + root[2:]
    if root == "$HOME" or root.startswith("$HOME/"):
        return f'"{root}{suffix}"'
    return shlex.quote(root + suffix)


def windows_to_wsl(path: Path) -> str:
    resolved = path.resolve()
    drive = resolved.drive.rstrip(":").lower()
    if not drive:
        raise ValueError(f"不是 Windows 绝对路径: {resolved}")
    remainder = resolved.as_posix()[3:]
    return f"/mnt/{drive}/{remainder}"


def ros_command(command: str) -> str:
    return (
        f"source {shlex.quote(ROS_SETUP)}; "
        f"source {workspace_shell_ref('/install/setup.bash')}; "
        f"{command}"
    )


def wsl_args(command: str, user: str | None = None) -> list[str]:
    args = [
        "wsl.exe",
        "-d",
        RUNTIME_CONFIG["wsl_distro"],
    ]
    if user:
        args.extend(("-u", user))
    args.extend([
        "--",
        "bash",
        "-lc",
        command,
    ])
    return args


class AcceptancePanel(tk.Tk):
    COLORS = {
        "bg": "#0f172a",
        "panel": "#172033",
        "panel_alt": "#1e293b",
        "text": "#f8fafc",
        "muted": "#94a3b8",
        "blue": "#38bdf8",
        "green": "#22c55e",
        "yellow": "#f59e0b",
        "red": "#ef4444",
        "border": "#334155",
    }

    TESTS = {
        "point": {
            "title": "点到点导航",
            "description": "直线响应、到点停车，验证 Nav2 与底盘闭环",
            "standard": (
                "① Nav2 返回 SUCCEEDED\n"
                "② 横向偏差 ≤ 0.08 m，路径比 ≤ 1.12\n"
                "③ 响应 ≤ 0.40 s、转向反复 ≤ 4 次，并自动回原点"
            ),
            "script": "measure_straight_motion.py",
            "args": (
                "--home-first --return-home --strict "
                "--goal-x 0.8 --goal-y 0.0 --timeout 100"
            ),
            "success": "STRAIGHT_RETURN_HOME_OK",
            "timeout": 230,
        },
        "dynamic": {
            "title": "随机动态避障鲁棒性",
            "description": "随机远终点；行驶后生成主障碍和同侧外圈随机障碍",
            "standard": (
                "① 连续 3 轮：随机终点距离 ≥ 2.00 m，位置误差 ≤ 0.30 m\n"
                "② 车先行驶 ≥ 0.25 m，再随机生成 1–4 个障碍物\n"
                "③ 障碍中心安全距离 ≥ 0.40 m，插入后至少重规划 1 次\n"
                "④ 有绕行（≥ 0.12 m）或至少 3 次安全降速\n"
                "⑤ 单次原地转向 ≤ 1.50 s、累计原地转角 ≤ 0.70 rad；倒车回原轨迹返航"
            ),
            "script": "validate_random_dynamic_avoidance.py",
            "args": "--cases 3",
            "success": "RANDOM_DYNAMIC_AVOIDANCE_VALIDATION_OK",
            "timeout": 700,
        },
        "patrol": {
            "title": "多点巡航",
            "description": "按 east → northeast → north → home 顺序连续导航",
            "standard": (
                "① 4 个航点按规定顺序全部完成，无失败\n"
                "② 航点 0.30 m 区域内转向反复 = 0，无效往返转角 ≤ 0.12 rad\n"
                "③ 队列清空并停车；回原点误差 ≤ 0.30 m"
            ),
            "script": "validate_task_patrol.py",
            "args": "--timeout 180",
            "success": "TASK_PATROL_VALIDATION_OK",
            "timeout": 210,
        },
        "random_patrol": {
            "title": "随机鲁棒巡航",
            "description": "每次生成 5 条不同方向与半径的安全闭环路线，可用 seed 复现",
            "standard": (
                "① 连续 5 条随机路线必须全部完成\n"
                "② 每条 4 点：航点 0.30 m 内反转 = 0，往返转角 ≤ 0.12 rad\n"
                "③ 每条回原点误差 ≤ 0.30 m；输出随机种子便于复现"
            ),
            "script": "validate_random_patrol.py",
            "args": "--cases 5",
            "success": "RANDOM_PATROL_VALIDATION_OK",
            "timeout": 760,
        },
        "vision_return": {
            "title": "真实视觉触发返航",
            "description": "到二维码站停车后授权；现场识别 NAV:HOME 自动返航",
            "standard": (
                "① 接近二维码站期间视觉授权关闭，到站停车后才开启\n"
                "② 授权后现场识别 NAV:HOME，自动生成 home 任务\n"
                "③ 队列清空并停车；回原点误差 ≤ 0.30 m"
            ),
            "script": "validate_vision_task_loop.py",
            "args": "",
            "success": "VISION_TASK_LOOP_VALIDATION_OK",
            "timeout": 190,
        },
    }

    def __init__(self) -> None:
        super().__init__()
        self.config = load_local_config()
        RUNTIME_CONFIG.update(self.config)
        self.title("智融行者 · ROS2 项目验收面板")
        # Keep the acceptance UI on the 24-inch secondary monitor to the
        # right of the 2560x1440 primary display.
        self.geometry("1120x1000+2860+380")
        self.minsize(980, 900)
        self.configure(bg=self.COLORS["bg"])

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.test_process: subprocess.Popen[str] | None = None
        self.system_process: subprocess.Popen[bytes] | None = None
        self.rviz_process: subprocess.Popen[bytes] | None = None
        self.busy_test: str | None = None
        self.result_labels: dict[str, tk.Label] = {}
        self.action_buttons: list[ttk.Button] = []
        self._log_tail_offsets: dict[Path, int] = {}
        self.setup_window: tk.Toplevel | None = None
        self.setup_vars: dict[str, tk.StringVar] = {}
        self.setup_buttons: list[ttk.Button] = []
        self.setup_status_var = tk.StringVar(value="等待操作")
        self.setup_busy = False

        self._configure_styles()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(120, self._drain_events)
        self.after(800, self._refresh_system_status_async)
        self.after(1200, self._tail_runtime_logs)

    def _configure_styles(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(
            "TButton",
            font=("Microsoft YaHei UI", 10, "bold"),
            padding=(14, 9),
            background=self.COLORS["panel_alt"],
            foreground=self.COLORS["text"],
            bordercolor=self.COLORS["border"],
        )
        style.map(
            "TButton",
            background=[("active", "#334155"), ("disabled", "#1e293b")],
            foreground=[("disabled", "#64748b")],
        )
        style.configure(
            "Primary.TButton",
            background="#0369a1",
            foreground="#ffffff",
        )
        style.map("Primary.TButton", background=[("active", "#0284c7")])
        style.configure(
            "Danger.TButton",
            background="#991b1b",
            foreground="#ffffff",
        )
        style.map("Danger.TButton", background=[("active", "#dc2626")])
        style.configure(
            "Horizontal.TProgressbar",
            background=self.COLORS["blue"],
            troughcolor=self.COLORS["panel_alt"],
            bordercolor=self.COLORS["panel_alt"],
        )

    def _build_ui(self) -> None:
        header = tk.Frame(self, bg=self.COLORS["bg"])
        header.pack(fill="x", padx=28, pady=(24, 14))
        header_left = tk.Frame(header, bg=self.COLORS["bg"])
        header_left.pack(side="left", fill="x", expand=True)
        tk.Label(
            header_left,
            text="智融行者 · 项目验收面板",
            font=("Microsoft YaHei UI", 22, "bold"),
            bg=self.COLORS["bg"],
            fg=self.COLORS["text"],
        ).pack(anchor="w")
        tk.Label(
            header_left,
            text="ROS2 Humble / Gazebo / RViz / Nav2 · 点击即可运行并记录结果",
            font=("Microsoft YaHei UI", 10),
            bg=self.COLORS["bg"],
            fg=self.COLORS["muted"],
        ).pack(anchor="w", pady=(5, 0))
        ttk.Button(
            header,
            text="跨机联调配置",
            command=self._open_machine_setup,
        ).pack(side="right", padx=(16, 0))

        status_frame = tk.Frame(
            self,
            bg=self.COLORS["panel"],
            highlightthickness=1,
            highlightbackground=self.COLORS["border"],
        )
        status_frame.pack(fill="x", padx=28, pady=(0, 14))
        status_left = tk.Frame(status_frame, bg=self.COLORS["panel"])
        status_left.pack(side="left", fill="x", expand=True, padx=18, pady=14)
        self.status_dot = tk.Label(
            status_left,
            text="●",
            font=("Segoe UI", 14),
            bg=self.COLORS["panel"],
            fg=self.COLORS["yellow"],
        )
        self.status_dot.pack(side="left")
        self.system_status = tk.Label(
            status_left,
            text="正在检查仿真状态…",
            font=("Microsoft YaHei UI", 11, "bold"),
            bg=self.COLORS["panel"],
            fg=self.COLORS["text"],
        )
        self.system_status.pack(side="left", padx=(8, 0))

        self.start_button = ttk.Button(
            status_frame,
            text="启动完整仿真",
            style="Primary.TButton",
            command=self._start_full_system,
        )
        self.start_button.pack(side="right", padx=(8, 18), pady=12)
        ttk.Button(
            status_frame,
            text="重新检查",
            command=self._refresh_system_status_async,
        ).pack(side="right", pady=12)

        cards = tk.Frame(self, bg=self.COLORS["bg"])
        cards.pack(fill="x", padx=28)
        for index, key in enumerate(("point", "dynamic", "patrol")):
            spec = self.TESTS[key]
            card = tk.Frame(
                cards,
                bg=self.COLORS["panel"],
                highlightthickness=1,
                highlightbackground=self.COLORS["border"],
            )
            card.grid(row=0, column=index, sticky="nsew", padx=(0, 10 if index < 2 else 0))
            cards.grid_columnconfigure(index, weight=1, uniform="test_cards")
            top = tk.Frame(card, bg=self.COLORS["panel"])
            top.pack(fill="x", padx=16, pady=(15, 6))
            tk.Label(
                top,
                text=f"0{index + 1}",
                font=("Segoe UI", 11, "bold"),
                bg=self.COLORS["panel"],
                fg=self.COLORS["blue"],
            ).pack(side="left")
            result = tk.Label(
                top,
                text="未运行",
                font=("Microsoft YaHei UI", 9, "bold"),
                bg=self.COLORS["panel_alt"],
                fg=self.COLORS["muted"],
                padx=9,
                pady=3,
            )
            result.pack(side="right")
            self.result_labels[key] = result
            tk.Label(
                card,
                text=spec["title"],
                font=("Microsoft YaHei UI", 15, "bold"),
                bg=self.COLORS["panel"],
                fg=self.COLORS["text"],
            ).pack(anchor="w", padx=16)
            tk.Label(
                card,
                text=spec["description"],
                font=("Microsoft YaHei UI", 9),
                bg=self.COLORS["panel"],
                fg=self.COLORS["muted"],
                wraplength=270,
                justify="left",
            ).pack(anchor="w", padx=16, pady=(7, 8))
            tk.Label(
                card,
                text="合格标准",
                font=("Microsoft YaHei UI", 9, "bold"),
                bg=self.COLORS["panel"],
                fg=self.COLORS["green"],
            ).pack(anchor="w", padx=16)
            tk.Label(
                card,
                text=spec["standard"],
                font=("Microsoft YaHei UI", 8),
                bg=self.COLORS["panel"],
                fg="#cbd5e1",
                wraplength=285,
                justify="left",
            ).pack(anchor="w", padx=16, pady=(4, 12))
            button = ttk.Button(
                card,
                text="开始验收",
                command=lambda name=key: self._run_test(name),
            )
            button.pack(fill="x", padx=16, pady=(0, 16))
            self.action_buttons.append(button)

        compact_tests = (
            (
                "04",
                "random_patrol",
                "标准：5/5 全部通过、零摆头、每次回原点",
                "开始随机验收",
            ),
            (
                "05",
                "vision_return",
                "标准：到站后授权、识别 NAV:HOME、返航误差 ≤ 0.30 m",
                "开始视觉验收",
            ),
        )
        for compact_number, key, short_standard, button_text in compact_tests:
            spec = self.TESTS[key]
            compact = tk.Frame(
                self,
                bg=self.COLORS["panel"],
                highlightthickness=1,
                highlightbackground=self.COLORS["border"],
            )
            compact.pack(fill="x", padx=28, pady=(10, 0))
            compact_text = tk.Frame(compact, bg=self.COLORS["panel"])
            compact_text.pack(side="left", fill="x", expand=True, padx=16, pady=9)
            compact_title = tk.Frame(compact_text, bg=self.COLORS["panel"])
            compact_title.pack(fill="x")
            tk.Label(
                compact_title,
                text=compact_number,
                font=("Segoe UI", 11, "bold"),
                bg=self.COLORS["panel"],
                fg=self.COLORS["blue"],
            ).pack(side="left")
            tk.Label(
                compact_title,
                text=spec["title"],
                font=("Microsoft YaHei UI", 13, "bold"),
                bg=self.COLORS["panel"],
                fg=self.COLORS["text"],
            ).pack(side="left", padx=(10, 0))
            result = tk.Label(
                compact_title,
                text="未运行",
                font=("Microsoft YaHei UI", 9, "bold"),
                bg=self.COLORS["panel_alt"],
                fg=self.COLORS["muted"],
                padx=9,
                pady=3,
            )
            result.pack(side="left", padx=(12, 0))
            self.result_labels[key] = result
            tk.Label(
                compact_text,
                text=f"{spec['description']}  ·  {short_standard}",
                font=("Microsoft YaHei UI", 9),
                bg=self.COLORS["panel"],
                fg=self.COLORS["muted"],
                justify="left",
            ).pack(anchor="w", pady=(4, 0))
            button = ttk.Button(
                compact,
                text=button_text,
                command=lambda name=key: self._run_test(name),
            )
            button.pack(side="right", padx=16, pady=12)
            self.action_buttons.append(button)

        controls = tk.Frame(self, bg=self.COLORS["bg"])
        controls.pack(fill="x", padx=28, pady=14)
        self.progress = ttk.Progressbar(controls, mode="indeterminate")
        self.progress.pack(side="left", fill="x", expand=True)
        self.current_action = tk.Label(
            controls,
            text="等待操作",
            font=("Microsoft YaHei UI", 9),
            bg=self.COLORS["bg"],
            fg=self.COLORS["muted"],
        )
        self.current_action.pack(side="left", padx=12)
        ttk.Button(
            controls,
            text="远程黑屏修复",
            command=self._reset_wslg,
        ).pack(side="right", padx=(8, 0))
        ttk.Button(
            controls,
            text="紧急停止",
            style="Danger.TButton",
            command=self._emergency_stop,
        ).pack(side="right")

        log_frame = tk.Frame(
            self,
            bg=self.COLORS["panel"],
            highlightthickness=1,
            highlightbackground=self.COLORS["border"],
        )
        log_frame.pack(fill="both", expand=True, padx=28, pady=(0, 24))
        log_header = tk.Frame(log_frame, bg=self.COLORS["panel"])
        log_header.pack(fill="x", padx=14, pady=(10, 6))
        tk.Label(
            log_header,
            text="运行记录",
            font=("Microsoft YaHei UI", 10, "bold"),
            bg=self.COLORS["panel"],
            fg=self.COLORS["text"],
        ).pack(side="left")
        ttk.Button(log_header, text="清空", command=self._clear_log).pack(side="right")
        self.log_text = tk.Text(
            log_frame,
            height=13,
            bg="#0b1220",
            fg="#cbd5e1",
            insertbackground="#ffffff",
            selectbackground="#1d4ed8",
            relief="flat",
            font=("Cascadia Mono", 9),
            padx=12,
            pady=10,
            wrap="word",
            state="disabled",
        )
        self.log_text.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self.log_text.tag_configure("ok", foreground=self.COLORS["green"])
        self.log_text.tag_configure("warn", foreground=self.COLORS["yellow"])
        self.log_text.tag_configure("error", foreground="#f87171")
        self.log_text.tag_configure("info", foreground=self.COLORS["blue"])
        self._append_log("验收面板已启动。请先确认系统状态，再运行验收。", "info")

    def _open_machine_setup(self) -> None:
        if self.setup_window is not None and self.setup_window.winfo_exists():
            self.setup_window.lift()
            self.setup_window.focus_force()
            return

        window = tk.Toplevel(self)
        self.setup_window = window
        window.title("跨机器联调 · 环境、源码与构建")
        window.geometry("880x650+3600+410")
        window.minsize(820, 600)
        window.configure(bg=self.COLORS["bg"])
        window.transient(self)

        body = tk.Frame(window, bg=self.COLORS["panel"])
        body.pack(fill="both", expand=True, padx=20, pady=20)
        tk.Label(
            body,
            text="跨机器联调配置",
            font=("Microsoft YaHei UI", 18, "bold"),
            bg=self.COLORS["panel"],
            fg=self.COLORS["text"],
        ).grid(row=0, column=0, columnspan=3, sticky="w", padx=18, pady=(16, 4))
        tk.Label(
            body,
            text=(
                "推荐顺序：保存 → 检测 → Clone/更新 → 安装基础环境 → "
                "安装项目依赖 → 构建。GitHub 发布仅上传已确认的源码。"
            ),
            font=("Microsoft YaHei UI", 9),
            bg=self.COLORS["panel"],
            fg=self.COLORS["muted"],
            wraplength=800,
            justify="left",
        ).grid(row=1, column=0, columnspan=3, sticky="w", padx=18, pady=(0, 14))

        self.setup_vars = {
            key: tk.StringVar(value=self.config[key]) for key in CONFIG_KEYS
        }
        commit_message = tk.StringVar(value="更新跨机器联调与验收功能")
        fields = (
            ("WSL 发行版", "wsl_distro", "例如 Ubuntu-22.04"),
            ("WSL 工作区", "wsl_workspace", "$HOME/zhirong_xingzhe_ws"),
            ("GitHub 仓库", "repo_url", "https://github.com/组织/仓库.git（不要带 Token）"),
            ("目标分支", "branch", "master"),
            ("Windows 源码目录", "source_root", "Clone 目标或现有项目根目录"),
        )
        for index, (label, key, hint) in enumerate(fields, start=2):
            tk.Label(
                body,
                text=label,
                font=("Microsoft YaHei UI", 9, "bold"),
                bg=self.COLORS["panel"],
                fg=self.COLORS["text"],
            ).grid(row=index, column=0, sticky="w", padx=(18, 10), pady=6)
            entry = ttk.Entry(body, textvariable=self.setup_vars[key])
            entry.grid(row=index, column=1, sticky="ew", pady=6)
            if key == "source_root":
                ttk.Button(
                    body,
                    text="选择目录",
                    command=self._browse_source_root,
                ).grid(row=index, column=2, padx=(8, 18), pady=6)
            else:
                tk.Label(
                    body,
                    text=hint,
                    font=("Microsoft YaHei UI", 8),
                    bg=self.COLORS["panel"],
                    fg=self.COLORS["muted"],
                ).grid(row=index, column=2, sticky="w", padx=(8, 18), pady=6)

        row = 7
        tk.Label(
            body,
            text="本次提交说明",
            font=("Microsoft YaHei UI", 9, "bold"),
            bg=self.COLORS["panel"],
            fg=self.COLORS["text"],
        ).grid(row=row, column=0, sticky="w", padx=(18, 10), pady=6)
        self.commit_message_var = commit_message
        ttk.Entry(body, textvariable=commit_message).grid(
            row=row, column=1, columnspan=2, sticky="ew", padx=(0, 18), pady=6
        )

        actions = tk.Frame(body, bg=self.COLORS["panel"])
        actions.grid(row=8, column=0, columnspan=3, sticky="ew", padx=18, pady=(14, 8))
        action_specs = (
            ("保存配置", self._save_setup_config),
            ("1 检测环境", self._detect_machine_environment),
            ("2 Clone / 安全更新", self._clone_or_update_source),
            ("3 安装基础环境", self._install_base_environment),
            ("4 安装项目依赖", self._install_project_dependencies),
            ("5 构建工作区", self._build_workspace),
            ("6 提交并推送 GitHub", self._publish_to_github),
        )
        self.setup_buttons = []
        for index, (text, command) in enumerate(action_specs):
            button = ttk.Button(actions, text=text, command=command)
            button.grid(row=index // 4, column=index % 4, sticky="ew", padx=4, pady=4)
            actions.grid_columnconfigure(index % 4, weight=1)
            self.setup_buttons.append(button)

        tk.Label(
            body,
            textvariable=self.setup_status_var,
            font=("Microsoft YaHei UI", 9, "bold"),
            bg=self.COLORS["panel_alt"],
            fg=self.COLORS["blue"],
            anchor="w",
            padx=12,
            pady=9,
        ).grid(row=9, column=0, columnspan=3, sticky="ew", padx=18, pady=(4, 10))
        tk.Label(
            body,
            text=(
                "安全说明：配置文件只保存在本机且不进 Git；仓库地址禁止内嵌账号、密码或 Token。"
                "系统安装和 GitHub 发布都会再次确认。"
            ),
            font=("Microsoft YaHei UI", 8),
            bg=self.COLORS["panel"],
            fg=self.COLORS["yellow"],
            wraplength=800,
            justify="left",
        ).grid(row=10, column=0, columnspan=3, sticky="w", padx=18, pady=(0, 16))
        body.grid_columnconfigure(1, weight=1)
        window.protocol("WM_DELETE_WINDOW", self._close_setup_window)

    def _close_setup_window(self) -> None:
        if self.setup_window is not None:
            self.setup_window.destroy()
        self.setup_window = None
        self.setup_buttons = []

    def _browse_source_root(self) -> None:
        initial = self.setup_vars.get("source_root", tk.StringVar()).get()
        selected = filedialog.askdirectory(
            parent=self.setup_window or self,
            title="选择现有项目目录或新的 Clone 目录",
            initialdir=initial if Path(initial).is_dir() else str(Path.home()),
        )
        if selected:
            self.setup_vars["source_root"].set(selected)

    @staticmethod
    def _validate_repo_url(repo_url: str, require_github: bool = False) -> None:
        if not repo_url:
            raise ValueError("请先填写 GitHub 仓库地址。")
        if any(char in repo_url for char in ("\r", "\n", "\x00")):
            raise ValueError("仓库地址包含非法控制字符。")
        parsed = urlsplit(repo_url)
        if parsed.scheme:
            if parsed.password or (
                parsed.scheme in ("http", "https") and parsed.username
            ):
                raise ValueError("仓库地址不能内嵌账号、密码或 Token。")
            host = (parsed.hostname or "").lower()
        else:
            ssh_match = re.match(r"^(?:[^@\s]+@)?([^:\s]+):.+$", repo_url)
            host = ssh_match.group(1).lower() if ssh_match else ""
        if require_github and host not in ("github.com", "www.github.com"):
            raise ValueError("GitHub 发布只接受 github.com 仓库地址。")

    def _config_from_form(self) -> dict[str, str]:
        if not self.setup_vars:
            return self.config.copy()
        config = {key: self.setup_vars[key].get().strip() for key in CONFIG_KEYS}
        if not re.fullmatch(r"[A-Za-z0-9._-]+", config["wsl_distro"]):
            raise ValueError("WSL 发行版名称不合法。")
        workspace = config["wsl_workspace"]
        if not workspace or any(
            token in workspace for token in ("\r", "\n", "\x00", '"', "`", "$(")
        ):
            raise ValueError("WSL 工作区路径不合法。")
        if not (
            workspace.startswith("/")
            or workspace == "~"
            or workspace.startswith("~/")
            or workspace == "$HOME"
            or workspace.startswith("$HOME/")
        ):
            raise ValueError("WSL 工作区必须是 Linux 绝对路径、~/... 或 $HOME/...")
        branch = config["branch"]
        if not branch or branch.startswith("-") or re.search(r"[\s~^:?*\\\[\]]", branch):
            raise ValueError("Git 分支名称不合法。")
        source_root = Path(config["source_root"]).expanduser()
        if not source_root.is_absolute():
            raise ValueError("Windows 源码目录必须是绝对路径。")
        config["source_root"] = str(source_root.resolve(strict=False))
        if config["repo_url"]:
            self._validate_repo_url(config["repo_url"])
        return config

    def _save_setup_config(self, quiet: bool = False) -> bool:
        try:
            config = self._config_from_form()
            CONFIG_PATH.write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            self.config = config
            RUNTIME_CONFIG.update(config)
            if not quiet:
                self.setup_status_var.set("配置已保存；后续启动和验收立即使用新配置。")
                self._append_log(f"跨机配置已保存：{CONFIG_PATH.name}", "ok")
            return True
        except (OSError, ValueError) as exc:
            messagebox.showerror("配置无效", str(exc), parent=self.setup_window or self)
            return False

    def _source_root(self) -> Path:
        return Path(self.config["source_root"])

    def _tools_dir(self) -> Path:
        return self._source_root() / "ros2_ws" / "tools"

    def _set_setup_busy(self, busy: bool, status: str = "等待操作") -> None:
        self.setup_busy = busy
        self.setup_status_var.set(status)
        for button in self.setup_buttons:
            button.configure(state="disabled" if busy else "normal")

    def _start_setup_job(self, label: str, operation: object) -> None:
        if self.setup_busy:
            return
        self._set_setup_busy(True, f"正在执行：{label}")
        self._append_log(f"跨机联调：{label}", "info")

        def worker() -> None:
            try:
                result = operation()  # type: ignore[operator]
                self.events.put(("setup_done", (label, str(result or "完成"))))
            except Exception as exc:
                self.events.put(("setup_failed", (label, str(exc))))

        threading.Thread(target=worker, daemon=True).start()

    def _run_streaming_process(
        self,
        args: list[str],
        timeout: int,
        cwd: Path | None = None,
    ) -> str:
        process = subprocess.Popen(
            args,
            cwd=str(cwd) if cwd else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            creationflags=CREATE_NO_WINDOW,
        )
        lines: list[str] = []
        assert process.stdout is not None
        deadline = time.monotonic() + timeout
        line_queue: queue.Queue[str | None] = queue.Queue()

        def read_output() -> None:
            assert process.stdout is not None
            for stream_line in process.stdout:
                line_queue.put(stream_line)
            line_queue.put(None)

        threading.Thread(target=read_output, daemon=True).start()
        stream_finished = False
        while not stream_finished:
            if time.monotonic() > deadline:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                raise RuntimeError(f"命令超过 {timeout} 秒，已停止。")
            try:
                line = line_queue.get(timeout=0.2)
            except queue.Empty:
                if process.poll() is not None:
                    stream_finished = True
                continue
            if line is None:
                stream_finished = True
                continue
            clean = line.replace("\x00", "").rstrip()
            if clean:
                lines.append(clean)
                self.events.put(("log", (clean, self._line_tag(clean))))
        return_code = process.wait(timeout=10)
        output = "\n".join(lines)
        if return_code != 0:
            tail = "\n".join(lines[-8:])
            raise RuntimeError(f"命令退出码 {return_code}。\n{tail}")
        return output

    def _detect_machine_environment(self) -> None:
        if not self._save_setup_config(quiet=True):
            return

        def operation() -> str:
            git_result = subprocess.run(
                ["git", "--version"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=10,
                creationflags=CREATE_NO_WINDOW,
            )
            git_status = git_result.stdout.strip() if git_result.returncode == 0 else "未安装"
            command = (
                "set +e; "
                "source /etc/os-release 2>/dev/null; "
                "echo OS=${PRETTY_NAME:-unknown}; "
                "if test -f /opt/ros/humble/setup.bash; then "
                "echo ROS2=OK; source /opt/ros/humble/setup.bash; else echo ROS2=MISSING; fi; "
                "command -v colcon >/dev/null && echo COLCON=OK || echo COLCON=MISSING; "
                "command -v rosdep >/dev/null && echo ROSDEP=OK || echo ROSDEP=MISSING; "
                "command -v gazebo >/dev/null && echo GAZEBO=OK || echo GAZEBO=MISSING; "
                "command -v rviz2 >/dev/null && echo RVIZ=OK || echo RVIZ=MISSING; "
                f"test -f {workspace_shell_ref('/install/setup.bash')} "
                "&& echo WORKSPACE=BUILT || echo WORKSPACE=NOT_BUILT"
            )
            code, output = self._run_capture(command, timeout=25)
            if code != 0:
                raise RuntimeError(output.strip() or "无法进入指定 WSL 发行版。")
            source_ok = (self._source_root() / "ros2_ws" / "src").is_dir()
            report = f"WINDOWS_GIT={git_status}\nSOURCE={'OK' if source_ok else 'MISSING'}\n{output.strip()}"
            for line in report.splitlines():
                self.events.put(("log", (line, "ok" if line.endswith(("OK", "BUILT")) else "warn")))
            return "环境检测完成；缺失项请按编号按钮补齐。"

        self._start_setup_job("检测 Windows / WSL / ROS2 / Gazebo / 工作区", operation)

    def _clone_or_update_source(self) -> None:
        if not self._save_setup_config(quiet=True):
            return
        try:
            self._validate_repo_url(self.config["repo_url"])
        except ValueError as exc:
            messagebox.showerror("缺少仓库地址", str(exc), parent=self.setup_window or self)
            return

        def operation() -> str:
            target = self._source_root()
            repo_url = self.config["repo_url"]
            branch = self.config["branch"]
            if (target / ".git").is_dir():
                dirty = subprocess.run(
                    ["git", "-C", str(target), "status", "--porcelain"],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=15,
                    creationflags=CREATE_NO_WINDOW,
                )
                if dirty.returncode != 0:
                    raise RuntimeError(dirty.stderr.strip() or "无法读取 Git 状态。")
                if dirty.stdout.strip():
                    raise RuntimeError("源码目录存在未提交修改；为防止覆盖，已拒绝更新。请先提交或备份。")
                self._run_streaming_process(
                    ["git", "-C", str(target), "fetch", "--prune", repo_url, branch],
                    timeout=180,
                )
                self._run_streaming_process(
                    ["git", "-C", str(target), "merge", "--ff-only", "FETCH_HEAD"],
                    timeout=60,
                )
                return f"源码已安全快进更新：{target}"
            if target.exists() and any(target.iterdir()):
                raise RuntimeError("Clone 目标目录非空且不是 Git 仓库，请选择空目录。")
            target.parent.mkdir(parents=True, exist_ok=True)
            self._run_streaming_process(
                [
                    "git", "clone", "--branch", branch, "--single-branch",
                    "--", repo_url, str(target),
                ],
                timeout=600,
            )
            return f"源码 Clone 完成：{target}"

        self._start_setup_job("Clone 或安全快进更新源码", operation)

    def _bootstrap_script(self) -> Path:
        script = self._tools_dir() / "bootstrap_machine.sh"
        if not script.is_file():
            raise FileNotFoundError(f"未找到跨机安装脚本：{script}")
        return script

    def _install_base_environment(self) -> None:
        if not self._save_setup_config(quiet=True):
            return
        warning = (
            "将以 root 身份修改指定 WSL 发行版，安装 ROS2 Humble、colcon、rosdep 等基础包。\n\n"
            "仅支持 Ubuntu 22.04；会联网下载并占用数 GB 空间。Windows 的 WSL 功能本身不会自动安装。\n\n"
            "是否继续？"
        )
        if not messagebox.askyesno("安装基础环境", warning, parent=self.setup_window or self):
            return

        def operation() -> str:
            script = windows_to_wsl(self._bootstrap_script())
            output = self._run_streaming_process(
                wsl_args(f"exec bash {shlex.quote(script)} --mode system", user="root"),
                timeout=1800,
            )
            if "SYSTEM_SETUP_OK" not in output:
                raise RuntimeError("安装脚本结束，但未返回 SYSTEM_SETUP_OK。")
            return "ROS2 基础环境安装完成。"

        self._start_setup_job("安装 WSL 内 ROS2 基础环境", operation)

    def _install_project_dependencies(self) -> None:
        if not self._save_setup_config(quiet=True):
            return
        if not messagebox.askyesno(
            "安装项目依赖",
            "将以 WSL root 身份运行 rosdep，可能通过 apt 安装项目依赖。是否继续？",
            parent=self.setup_window or self,
        ):
            return

        def operation() -> str:
            script = windows_to_wsl(self._bootstrap_script())
            source = windows_to_wsl(self._source_root() / "ros2_ws" / "src")
            output = self._run_streaming_process(
                wsl_args(
                    f"exec bash {shlex.quote(script)} --mode dependencies "
                    f"--source {shlex.quote(source)}",
                    user="root",
                ),
                timeout=1800,
            )
            if "PROJECT_DEPENDENCIES_OK" not in output:
                raise RuntimeError("依赖安装结束，但未返回 PROJECT_DEPENDENCIES_OK。")
            return "项目依赖安装完成。"

        self._start_setup_job("用 rosdep 安装项目依赖", operation)

    def _build_workspace(self) -> None:
        if not self._save_setup_config(quiet=True):
            return

        def operation() -> str:
            setup_script = self._tools_dir() / "setup_workspace.sh"
            if not setup_script.is_file():
                raise FileNotFoundError(f"未找到工作区脚本：{setup_script}")
            command = (
                f"export ZHIRONG_WORKSPACE={workspace_shell_ref()}; "
                f"exec bash {shlex.quote(windows_to_wsl(setup_script))}"
            )
            output = self._run_streaming_process(wsl_args(command), timeout=1200)
            if "SETUP_WORKSPACE_OK" not in output:
                raise RuntimeError("构建结束，但未返回 SETUP_WORKSPACE_OK。")
            return "colcon 构建完成；可返回主面板启动仿真。"

        self._start_setup_job("链接源码并 colcon 构建", operation)

    @staticmethod
    def _sensitive_git_paths(paths: list[str]) -> list[str]:
        sensitive_names = {
            ".env", "credentials", "credentials.json", "secrets.json",
            "id_rsa", "id_ed25519", ".netrc", "known_hosts",
            ".acceptance_panel.local.json",
        }
        sensitive_suffixes = (".pem", ".key", ".p12", ".pfx", ".kdbx")
        blocked: list[str] = []
        for raw in paths:
            path = raw.split(" -> ")[-1].replace("\\", "/")
            name = Path(path).name.lower()
            if name.endswith(".example") or name.endswith(".sample"):
                continue
            if name in sensitive_names or name.endswith(sensitive_suffixes):
                blocked.append(path)
        return blocked

    @staticmethod
    def _git_changed_paths(porcelain_z: str) -> list[str]:
        entries = porcelain_z.split("\x00")
        paths: list[str] = []
        index = 0
        while index < len(entries):
            entry = entries[index]
            if len(entry) < 4:
                index += 1
                continue
            status = entry[:2]
            paths.append(entry[3:])
            index += 2 if "R" in status or "C" in status else 1
        return paths

    @staticmethod
    def _secret_content_paths(root: Path, paths: list[str]) -> list[str]:
        patterns = (
            re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
            re.compile(rb"github_pat_[A-Za-z0-9_]{20,}"),
            re.compile(rb"ghp_[A-Za-z0-9]{30,}"),
            re.compile(rb"AKIA[0-9A-Z]{16}"),
        )
        blocked: list[str] = []
        root_resolved = root.resolve()
        for relative in paths:
            candidate = (root / relative).resolve(strict=False)
            try:
                candidate.relative_to(root_resolved)
            except ValueError:
                blocked.append(relative)
                continue
            try:
                if not candidate.is_file() or candidate.stat().st_size > 2_000_000:
                    continue
                content = candidate.read_bytes()
            except OSError:
                continue
            if any(pattern.search(content) for pattern in patterns):
                blocked.append(relative)
        return blocked

    def _publish_to_github(self) -> None:
        if not self._save_setup_config(quiet=True):
            return
        try:
            self._validate_repo_url(self.config["repo_url"], require_github=True)
        except ValueError as exc:
            messagebox.showerror("GitHub 地址无效", str(exc), parent=self.setup_window or self)
            return
        root = self._source_root()
        if not (root / ".git").is_dir():
            messagebox.showerror("不是 Git 仓库", "请先 Clone 源码或选择已有 Git 项目。", parent=self.setup_window or self)
            return
        try:
            status = subprocess.run(
                [
                    "git", "-C", str(root), "status", "--porcelain=v1", "-z",
                    "--untracked-files=all",
                ],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=20,
                creationflags=CREATE_NO_WINDOW,
            )
            if status.returncode != 0:
                raise RuntimeError(status.stderr.strip() or "无法读取 Git 状态。")
            changed_paths = self._git_changed_paths(status.stdout)
            blocked = sorted(set(
                self._sensitive_git_paths(changed_paths)
                + self._secret_content_paths(root, changed_paths)
            ))
            if blocked:
                raise RuntimeError("发现疑似敏感文件，已阻止上传：\n" + "\n".join(blocked[:12]))
            commit_message = self.commit_message_var.get().strip()
            if changed_paths and not commit_message:
                raise RuntimeError("存在待提交文件，请填写本次提交说明。")
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            messagebox.showerror("发布前检查失败", str(exc), parent=self.setup_window or self)
            return

        preview = "\n".join(changed_paths[:12]) if changed_paths else "（没有未提交修改，仅推送现有提交）"
        if len(changed_paths) > 12:
            preview += f"\n……另有 {len(changed_paths) - 12} 个文件"
        warning = (
            f"目标：{self.config['repo_url']}\n"
            f"分支：{self.config['branch']}\n\n"
            f"待提交/推送：\n{preview}\n\n"
            "确认后将配置 origin、提交上述修改并推送到 GitHub。此操作会影响远端仓库。"
        )
        if not messagebox.askyesno("确认 GitHub 发布", warning, parent=self.setup_window or self):
            return

        def operation() -> str:
            if changed_paths:
                self._run_streaming_process(["git", "-C", str(root), "add", "-A"], timeout=60)
                self._run_streaming_process(
                    ["git", "-C", str(root), "commit", "-m", commit_message], timeout=120
                )
            remote = subprocess.run(
                ["git", "-C", str(root), "remote", "get-url", "origin"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=15,
                creationflags=CREATE_NO_WINDOW,
            )
            remote_command = "set-url" if remote.returncode == 0 else "add"
            self._run_streaming_process(
                ["git", "-C", str(root), "remote", remote_command, "origin", self.config["repo_url"]],
                timeout=30,
            )
            output = self._run_streaming_process(
                [
                    "git", "-C", str(root), "push", "-u", "origin",
                    f"HEAD:refs/heads/{self.config['branch']}",
                ],
                timeout=600,
            )
            return "源码已提交并推送到 GitHub。" if changed_paths else "现有提交已推送到 GitHub。"

        self._start_setup_job("提交并推送 GitHub", operation)

    def _append_log(self, message: str, tag: str | None = None) -> None:
        clean = message.replace("\x00", "").rstrip()
        if not clean:
            return
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {clean}\n", tag or "")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _set_system_status(self, text: str, state: str) -> None:
        color = {
            "ready": self.COLORS["green"],
            "checking": self.COLORS["yellow"],
            "error": self.COLORS["red"],
        }[state]
        self.system_status.configure(text=text)
        self.status_dot.configure(fg=color)

    def _set_test_result(self, name: str, text: str, state: str) -> None:
        colors = {
            "idle": (self.COLORS["panel_alt"], self.COLORS["muted"]),
            "running": ("#78350f", "#fde68a"),
            "passed": ("#14532d", "#bbf7d0"),
            "failed": ("#7f1d1d", "#fecaca"),
        }
        bg, fg = colors[state]
        self.result_labels[name].configure(text=text, bg=bg, fg=fg)

    def _set_busy(self, name: str | None) -> None:
        self.busy_test = name
        if name is None:
            self.progress.stop()
            self.current_action.configure(text="等待操作")
            for button in self.action_buttons:
                button.configure(state="normal")
            self.start_button.configure(state="normal")
        else:
            self.progress.start(12)
            self.current_action.configure(text=f"正在运行：{self.TESTS[name]['title']}")
            for button in self.action_buttons:
                button.configure(state="disabled")
            self.start_button.configure(state="disabled")

    def _run_capture(self, command: str, timeout: int = 15) -> tuple[int, str]:
        completed = subprocess.run(
            wsl_args(command),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            creationflags=CREATE_NO_WINDOW,
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        return completed.returncode, output.replace("\x00", "")

    def _refresh_system_status_async(self) -> None:
        self._set_system_status("正在检查 Gazebo / RViz / Nav2…", "checking")

        def worker() -> None:
            command = ros_command(
                "if pgrep -x gzserver >/dev/null "
                "&& pgrep -x gzclient >/dev/null "
                "&& pgrep -x rviz2 >/dev/null "
                "&& ros2 action list 2>/dev/null | grep -qx /navigate_to_pose; "
                "then echo SYSTEM_READY; else echo SYSTEM_NOT_READY; fi"
            )
            try:
                _, output = self._run_capture(command, timeout=20)
                ready = "SYSTEM_READY" in output
                self.events.put(("system_status", ready))
            except Exception as exc:
                self.events.put(("system_error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _start_full_system(self) -> None:
        self.start_button.configure(state="disabled")
        self._set_system_status("正在启动完整仿真，通常需要 20–40 秒…", "checking")
        self._append_log("开始启动 ROS2 + Gazebo + Nav2。", "info")

        def worker() -> None:
            try:
                check_command = ros_command(
                    "if pgrep -x gzserver >/dev/null "
                    "&& ros2 action list 2>/dev/null | grep -qx /navigate_to_pose; "
                    "then echo BACKEND_READY; fi"
                )
                _, check_output = self._run_capture(check_command, timeout=15)
                backend_ready = "BACKEND_READY" in check_output

                if not backend_ready:
                    system_log = LOG_DIR / "full_system.log"
                    system_handle = open(system_log, "ab", buffering=0)
                    command = ros_command(
                        "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA; "
                        "export GAZEBO_MODEL_DATABASE_URI=; "
                        f"export GAZEBO_MODEL_PATH={workspace_shell_ref('/install/zhirong_gazebo/share/zhirong_gazebo/models')}; "
                        "exec ros2 launch zhirong_bringup system.launch.py "
                        "gui:=true navigation_rviz:=false"
                    )
                    self.system_process = subprocess.Popen(
                        wsl_args(command),
                        stdout=system_handle,
                        stderr=subprocess.STDOUT,
                        creationflags=CREATE_NO_WINDOW,
                    )
                    self.events.put(("log", ("完整仿真后端已发起。", "info")))

                deadline = time.monotonic() + 55
                nav_ready = False
                while time.monotonic() < deadline:
                    time.sleep(2)
                    _, output = self._run_capture(
                        ros_command(
                            "ros2 action list 2>/dev/null | "
                            "grep -qx /navigate_to_pose && echo NAV_READY"
                        ),
                        timeout=10,
                    )
                    if "NAV_READY" in output:
                        nav_ready = True
                        break
                if not nav_ready:
                    raise RuntimeError("55 秒内未检测到 /navigate_to_pose。")

                _, rviz_output = self._run_capture(
                    "pgrep -x rviz2 >/dev/null && echo RVIZ_READY || true",
                    timeout=10,
                )
                if "RVIZ_READY" not in rviz_output:
                    rviz_log = LOG_DIR / "rviz.log"
                    rviz_handle = open(rviz_log, "ab", buffering=0)
                    rviz_command = ros_command(
                        "export LIBGL_ALWAYS_SOFTWARE=1; "
                        "export QT_X11_NO_MITSHM=1; "
                        f"exec rviz2 -d {workspace_shell_ref('/install/zhirong_bringup/share/zhirong_bringup/rviz/zhirong_navigation.rviz')} --ros-args "
                        "-r __node:=rviz2_acceptance_panel -p use_sim_time:=true"
                    )
                    self.rviz_process = subprocess.Popen(
                        wsl_args(rviz_command),
                        stdout=rviz_handle,
                        stderr=subprocess.STDOUT,
                        creationflags=CREATE_NO_WINDOW,
                    )
                    self.events.put(("log", ("RViz 可视化窗口已发起。", "info")))
                    time.sleep(5)

                self.events.put(("system_started", None))
            except Exception as exc:
                self.events.put(("system_error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _run_test(self, name: str) -> None:
        if self.busy_test is not None:
            return
        spec = self.TESTS[name]
        self._set_test_result(name, "运行中", "running")
        self._set_busy(name)
        self._append_log(f"开始验收：{spec['title']}。", "info")
        self._append_log(
            "合格标准：" + str(spec["standard"]).replace("\n", "；"),
            "info",
        )

        def worker() -> None:
            try:
                ready_command = ros_command(
                    "ros2 action list 2>/dev/null | "
                    "grep -qx /navigate_to_pose && echo READY"
                )
                _, output = self._run_capture(ready_command, timeout=15)
                if "READY" not in output:
                    raise RuntimeError("Nav2 尚未就绪，请先点击“启动完整仿真”。")

                script_path = windows_to_wsl(self._tools_dir() / str(spec["script"]))
                test_invocation = (
                    f"python3 -u {shlex.quote(script_path)} {spec['args']}"
                )
                command = ros_command(
                    "export PYTHONUNBUFFERED=1; "
                    f"exec bash -lc {shlex.quote(test_invocation)}"
                )
                process = subprocess.Popen(
                    wsl_args(command),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    creationflags=CREATE_NO_WINDOW,
                )
                self.test_process = process
                lines: list[str] = []
                assert process.stdout is not None
                deadline = time.monotonic() + int(spec["timeout"])
                line_queue: queue.Queue[str | None] = queue.Queue()

                def read_output() -> None:
                    assert process.stdout is not None
                    for stream_line in process.stdout:
                        line_queue.put(stream_line)
                    line_queue.put(None)

                threading.Thread(target=read_output, daemon=True).start()
                stream_finished = False
                while not stream_finished:
                    if time.monotonic() > deadline:
                        process.terminate()
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            process.kill()
                        raise RuntimeError("验收超时，测试进程已停止。")
                    try:
                        line = line_queue.get(timeout=0.2)
                    except queue.Empty:
                        if process.poll() is not None:
                            stream_finished = True
                        continue
                    if line is None:
                        stream_finished = True
                    else:
                        clean = line.replace("\x00", "").rstrip()
                        lines.append(clean)
                        self.events.put(("log", (clean, self._line_tag(clean))))
                return_code = process.wait(timeout=5)
                output_text = "\n".join(lines)
                if return_code != 0 or str(spec["success"]) not in output_text:
                    raise RuntimeError(
                        f"未检测到 {spec['success']}（退出码 {return_code}）。"
                    )
                self.events.put(("test_passed", name))
            except Exception as exc:
                self.events.put(("test_failed", (name, str(exc))))
            finally:
                self.test_process = None

        threading.Thread(target=worker, daemon=True).start()

    @staticmethod
    def _line_tag(line: str) -> str | None:
        upper = line.upper()
        if upper.endswith("_OK") or "SUCCEEDED" in upper:
            return "ok"
        if "ERROR" in upper or "TRACEBACK" in upper or "FAILED" in upper:
            return "error"
        if "WARN" in upper:
            return "warn"
        if "INSERTED" in upper or "CLEARED" in upper or "PROGRESS" in upper:
            return "info"
        return None

    def _emergency_stop(self) -> None:
        self._append_log("执行紧急停止：取消当前验收并发送零速度。", "warn")
        if self.test_process is not None and self.test_process.poll() is None:
            try:
                self.test_process.terminate()
            except OSError:
                pass
        self.test_process = None
        current = self.busy_test
        if current:
            self._set_test_result(current, "已中止", "failed")
        self._set_busy(None)

        def worker() -> None:
            script_path = windows_to_wsl(self._tools_dir() / "emergency_stop.py")
            command = ros_command(
                "pkill -INT -f auto_navigate_test.py 2>/dev/null || true; "
                "pkill -INT -f validate_dynamic_avoidance.py 2>/dev/null || true; "
                "pkill -INT -f validate_random_dynamic_avoidance.py 2>/dev/null || true; "
                "pkill -INT -f validate_task_patrol.py 2>/dev/null || true; "
                f"python3 -u {shlex.quote(script_path)}"
            )
            try:
                code, output = self._run_capture(command, timeout=15)
                self.events.put(("log", (output.strip(), "ok" if code == 0 else "error")))
            except Exception as exc:
                self.events.put(("log", (f"紧急停止执行异常：{exc}", "error")))

        threading.Thread(target=worker, daemon=True).start()

    def _reset_wslg(self) -> None:
        warning = (
            "该操作用于修复远程控制时 Gazebo/RViz 有窗口但不可见的问题。\n\n"
            "它会停止所有 WSL 发行版中的进程，包括正在运行的仿真或 Docker。"
            "Windows 和远程连接不会关闭。\n\n是否继续？"
        )
        if not messagebox.askyesno("重置 WSLg", warning, parent=self):
            return
        self._append_log("正在执行 wsl --shutdown；完成后请重新启动完整仿真。", "warn")
        self._set_system_status("正在重置 WSLg…", "checking")

        def worker() -> None:
            try:
                completed = subprocess.run(
                    ["wsl.exe", "--shutdown"],
                    capture_output=True,
                    timeout=25,
                    creationflags=CREATE_NO_WINDOW,
                )
                if completed.returncode != 0:
                    raise RuntimeError("wsl --shutdown 返回失败。")
                self.events.put(("wsl_reset", None))
            except Exception as exc:
                self.events.put(("system_error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _tail_runtime_logs(self) -> None:
        for path in (LOG_DIR / "full_system.log", LOG_DIR / "rviz.log"):
            try:
                if not path.exists():
                    continue
                offset = self._log_tail_offsets.get(path, 0)
                size = path.stat().st_size
                if size < offset:
                    offset = 0
                if size > offset:
                    with path.open("r", encoding="utf-8", errors="replace") as stream:
                        stream.seek(offset)
                        new_lines = stream.readlines()[-8:]
                        self._log_tail_offsets[path] = stream.tell()
                    for line in new_lines:
                        if any(token in line for token in ("ERROR", "process has died")):
                            self._append_log(line.strip(), "error")
            except OSError:
                pass
        self.after(1500, self._tail_runtime_logs)

    def _drain_events(self) -> None:
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "log":
                message, tag = payload  # type: ignore[misc]
                self._append_log(str(message), tag)
            elif event == "system_status":
                if bool(payload):
                    self._set_system_status("完整仿真已就绪，可开始验收", "ready")
                else:
                    self._set_system_status("仿真未完整启动", "error")
                self.start_button.configure(state="normal")
            elif event == "system_started":
                self._set_system_status("完整仿真已就绪，可开始验收", "ready")
                self.start_button.configure(state="normal")
                self._append_log("Gazebo、RViz 和 Nav2 已就绪。", "ok")
            elif event == "system_error":
                self._set_system_status("启动或检查失败，请查看日志", "error")
                self.start_button.configure(state="normal")
                self._append_log(str(payload), "error")
            elif event == "test_passed":
                name = str(payload)
                self._set_test_result(name, "✓ 合格", "passed")
                self._append_log(
                    f"{self.TESTS[name]['title']}：全部合格标准已满足。",
                    "ok",
                )
                self._set_busy(None)
            elif event == "test_failed":
                name, error = payload  # type: ignore[misc]
                self._set_test_result(str(name), "失败", "failed")
                self._append_log(f"{self.TESTS[str(name)]['title']}：{error}", "error")
                self._set_busy(None)
            elif event == "wsl_reset":
                self._set_system_status("WSLg 已重置，请点击“启动完整仿真”", "error")
                self._append_log("WSLg 重置完成。", "ok")
            elif event == "setup_done":
                label, message = payload  # type: ignore[misc]
                self._set_setup_busy(False, str(message))
                self._append_log(f"{label}：{message}", "ok")
            elif event == "setup_failed":
                label, error = payload  # type: ignore[misc]
                self._set_setup_busy(False, f"失败：{label}")
                self._append_log(f"{label}：{error}", "error")
                if self.setup_window is not None and self.setup_window.winfo_exists():
                    messagebox.showerror(
                        "跨机联调步骤失败",
                        f"{label}\n\n{error}",
                        parent=self.setup_window,
                    )
        self.after(120, self._drain_events)

    def _on_close(self) -> None:
        if self.busy_test is not None:
            if not messagebox.askyesno(
                "验收仍在运行",
                "关闭面板会中止当前验收并发送紧急停止。是否关闭？",
                parent=self,
            ):
                return
            self._emergency_stop()
        self.destroy()


def self_test() -> int:
    required = [
        TOOLS_DIR / "auto_navigate_test.py",
        TOOLS_DIR / "measure_straight_motion.py",
        TOOLS_DIR / "validate_dynamic_avoidance.py",
        TOOLS_DIR / "validate_random_dynamic_avoidance.py",
        TOOLS_DIR / "validate_task_patrol.py",
        TOOLS_DIR / "validate_vision_task_loop.py",
        TOOLS_DIR / "emergency_stop.py",
        TOOLS_DIR / "bootstrap_machine.sh",
        TOOLS_DIR / "setup_workspace.sh",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    payload = {
        "project_root": str(PROJECT_ROOT),
        "tools_dir": str(TOOLS_DIR),
        "wsl_tools_dir": windows_to_wsl(TOOLS_DIR),
        "required_scripts_ok": not missing,
        "missing": missing,
        "default_config": DEFAULT_CONFIG,
        "runtime_workspace_setup": workspace_shell_ref("/install/setup.bash"),
        "tk_version": tk.TkVersion,
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if not missing else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    app = AcceptancePanel()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
