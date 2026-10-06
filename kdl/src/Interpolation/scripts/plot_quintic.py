#!/usr/bin/env python3
# Copyright (c) 2026, kdl_interpolation authors.
# 教学用途：把 quintic_demo 导出的 CSV 画成曲线图（总览 + 分张）。
#
# 运行方式：
#   ros2 run kdl_tools plot_quintic                       # 读 ./quintic_trajectory.csv
#   ros2 run kdl_tools plot_quintic path/to/traj.csv      # 指定 CSV
#   ros2 run kdl_tools plot_quintic traj.csv --out-dir /tmp/plots
#   ros2 run kdl_tools plot_quintic traj.csv --show       # 额外弹出交互窗口
#
# 输出（默认写到 CSV 所在目录，<名字> 取自 CSV 文件名）：
#   <名字>_overview.png   2×3 总览：position / velocity / acceleration / jerk / snap + 文字信息
#   <名字>_q.png          每个量再单独存一张，方便放大看细节
#   <名字>_qdot.png  ...
#
# 为什么用 Python 画、而不是在 C++ 里画：
#   CSV 是"数据的落地格式"，画图是"看数据"的事，两者职责不同。让 C++ 只负责
#   算与存、Python 只负责读与画，与本包一贯的分工一致（见各模块的 *_print.hpp：
#   求解逻辑不需要知道怎么打印）。
#
# 为什么图里的文字用英文：
#   matplotlib 自带的 DejaVu Sans 没有中文字形，中文标签会显示成方框。
#   要在图里写中文，需要另外指定系统中文字体——那属于环境配置，不该由这个
#   教学脚本承担，所以图内一律英文，终端提示保持中文。

import argparse
import os
import sys

import numpy as np
import matplotlib

# 默认切到无窗口后端：这样在没有显示器的环境（ssh、CI、容器）里也能直接跑出 PNG。
# 只有显式传了 --show，才让 matplotlib 自己挑一个能弹窗的后端。
if "--show" not in sys.argv:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  （必须在选好后端之后再导入 pyplot）

# ---------------------------------------------------------------------------
# 要画的五个量：CSV 列前缀 -> (图标题, 纵轴标签)
#
# 纵轴标签刻意写成 dq/dt 这种形式而不是 q̇：一是避免依赖字体里的组合点字符，
# 二是把"这是对时间求导"这件事写明白——jerk/snap 最容易被误解成别的量。
#
# 脚本只画 CSV 里**真实存在**的列：若 CSV 没有 jerk/snap 列（老版本导出），
# 会自动少画一格而不是直接报错——画图工具没资格要求数据必须长成什么样。
# ---------------------------------------------------------------------------
PANELS = [
    ("q", "Joint position", "q [rad]"),
    ("qdot", "Joint velocity", "dq/dt [rad/s]"),
    ("qddot", "Joint acceleration", "d2q/dt2 [rad/s^2]"),
    ("jerk", "Joint jerk (3rd derivative)", "d3q/dt3 [rad/s^3]"),
    ("snap", "Joint snap (4th derivative)", "d4q/dt4 [rad/s^4]"),
]


def parse_args(argv=None):
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="把 quintic_demo 导出的 CSV 画成曲线图（总览 + 分张 PNG）。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "csv",
        nargs="?",
        default="quintic_trajectory.csv",
        help="quintic_demo 导出的 CSV 路径",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="图片输出目录（缺省为 CSV 所在目录）",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="除保存 PNG 外再弹出交互窗口（需要图形环境）",
    )
    return parser.parse_args(argv)


def find_series(data, prefix):
    """在 CSV 表头里找出所有 ``prefix<数字>`` 形式的列。

    :param data: ``np.genfromtxt(..., names=True)`` 的结果
    :param prefix: 列前缀，例如 ``"q"``
    :return: ``[(关节号, 列名), ...]``，按关节号升序

    前缀 ``"q"`` 同时也能匹配 ``"qdot0"`` 的开头，所以这里要求剩余部分是
    **纯数字**，否则 ``qdot0`` 会被误当成关节 0 的位置列。
    """
    found = []
    for name in data.dtype.names or ():
        if not name.startswith(prefix):
            continue
        suffix = name[len(prefix):]
        if suffix.isdigit():
            found.append((int(suffix), name))
    return sorted(found)


def load_csv(path):
    """读入 CSV，返回结构化数组（表头即字段名）。"""
    data = np.genfromtxt(path, delimiter=",", names=True)
    # 只有一行数据时 genfromtxt 会返回 0 维数组，统一拉平，后面就不必到处特判。
    return np.atleast_1d(data)


def available_panels(data):
    """筛出 CSV 里真正有数据的那些量，返回 PANELS 的子集。"""
    return [panel for panel in PANELS if find_series(data, panel[0])]


def build_info_text(data, csv_path):
    """拼出总览图最后一格里的文字信息：采样规模 + 各量的峰值。"""
    t = np.asarray(data["t"], dtype=float)
    dt = float(np.mean(np.diff(t))) if t.size > 1 else 0.0

    lines = [
        "file    : {}".format(os.path.basename(csv_path)),
        "samples : {}".format(t.size),
        "span    : {:.4f} s".format(float(t[-1] - t[0])),
        "dt      : {:.4f} s".format(dt),
        "",
    ]
    for prefix, _, _ in available_panels(data):
        columns = [column for _, column in find_series(data, prefix)]
        values = np.column_stack([data[column] for column in columns])
        lines.append("peak |{:<6}| = {:>12.5g}".format(prefix, float(np.max(np.abs(values)))))
    return "\n".join(lines)


def draw_panel(ax, data, prefix, title, ylabel):
    """在给定坐标轴上画出该量的所有关节曲线。"""
    for joint, column in find_series(data, prefix):
        ax.plot(data["t"], data[column], label="joint {}".format(joint))
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def build_overview(data, panels, csv_path):
    """构造 2×3 总览图：前几格画曲线，最后一格放文字信息。"""
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    flat = axes.ravel()

    for index, (prefix, title, ylabel) in enumerate(panels):
        draw_panel(flat[index], data, prefix, title, ylabel)
        if index == 0:
            # 图例只在第一格给一次：各关节的颜色映射在全部子图里是一致的。
            flat[index].legend(fontsize=8, ncol=2)

    # x 轴共享，所以只在最下面一行标注时间轴。
    for ax in flat[3:6]:
        ax.set_xlabel("t [s]")

    # 剩下没用到的格子：第一格写文字信息，其余直接隐藏。
    for ax in flat[len(panels) + 1:]:
        ax.axis("off")
    if len(panels) < flat.size:
        info_ax = flat[len(panels)]
        info_ax.axis("off")
        info_ax.text(
            0.0, 1.0, build_info_text(data, csv_path), va="top", ha="left",
            family="monospace", fontsize=10,
        )

    fig.suptitle("Quintic joint-space interpolation — {}".format(os.path.basename(csv_path)))
    fig.tight_layout()
    return fig


def build_single(data, prefix, title, ylabel):
    """构造单个量的图，方便放大看细节。"""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    draw_panel(ax, data, prefix, title, ylabel)
    ax.set_xlabel("t [s]")
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    return fig


def main(argv=None):
    """读 CSV、画图、保存，并按需弹窗。"""
    args = parse_args(argv)

    if not os.path.isfile(args.csv):
        print("找不到 CSV 文件: {}".format(args.csv))
        print("提示: 先运行 `ros2 run kdl_tools quintic_demo` 生成它"
              "（CSV 会落在运行时的工作目录里）。")
        return 1

    data = load_csv(args.csv)
    panels = available_panels(data)
    if not panels:
        print("CSV 里没有任何可画的列（期望 t 与 q0..qN 这种命名）: {}".format(args.csv))
        return 1

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(out_dir, exist_ok=True)

    # 输出文件名前缀取自 CSV 的文件名，这样同一个目录里放多条轨迹也不会互相覆盖。
    stem = os.path.splitext(os.path.basename(args.csv))[0]

    written = []
    figures = []

    overview = build_overview(data, panels, args.csv)
    overview_path = os.path.join(out_dir, "{}_overview.png".format(stem))
    overview.savefig(overview_path, bbox_inches="tight")
    figures.append(overview)
    written.append(overview_path)

    for prefix, title, ylabel in panels:
        figure = build_single(data, prefix, title, ylabel)
        path = os.path.join(out_dir, "{}_{}.png".format(stem, prefix))
        figure.savefig(path, bbox_inches="tight")
        figures.append(figure)
        written.append(path)

    print("已读取 {} 个采样点、{} 个关节、{} 个量:".format(
        len(data["t"]), len(find_series(data, "q")), len(panels)))
    for path in written:
        print("  {}".format(os.path.abspath(path)))

    if args.show:
        plt.show()

    for figure in figures:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    sys.exit(main())
