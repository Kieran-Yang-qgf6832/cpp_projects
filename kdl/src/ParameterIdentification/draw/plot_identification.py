#!/usr/bin/env python3
# Copyright (c) 2026, kdl_identification authors.
# 教学用途：把 kdl_identify_node 导出的两份 CSV 画成图（总览 + 分关节 + 参数）。
#
# 运行方式：
#   ros2 run kdl_tools plot_identification                    # 读 <draw>/identification_samples.csv
#   ros2 run kdl_tools plot_identification path/samples.csv   # 指定样本 CSV
#   ros2 run kdl_tools plot_identification --out-dir /tmp/plots
#   ros2 run kdl_tools plot_identification --show             # 额外弹出交互窗口
#
# 输入：
#   <samples>.csv  kdl_identify_node 导出的样本：t, q0.., qdot0.., qddot0.., tau0..[, tau_pred0..]
#   <params>.csv   同一节点导出的参数：index, label, value
#
# 输入输出默认都落在 src/ParameterIdentification/draw（即本脚本所在目录）：
#   节点把样本/参数 CSV 写在那里，本脚本把 PNG 也写在那里。
#
# 输出：
#   identification_overview.png              2x3 总览：q / qdot / qddot / tau(实测 vs 预测) /
#                                            残差 / 文字信息
#   identification_torque_jointN.png         每个关节一张：上=力矩对比，下=残差
#   identification_parameters.png            基参数（含摩擦项）水平条形图
#   identification_friction.png              各关节粘性/库伦摩擦对照
#
# 为什么图里文字用英文：matplotlib 自带的 DejaVu Sans 没有中文字形，中文会变方框。
# 与 plot_quintic / plot_cartesian 保持同一约定：图内英文，终端提示中文。

import argparse
import csv as csv_module
import os
import sys

import numpy as np
import matplotlib

# 默认无窗口后端：ssh / CI / 容器里也能直接出 PNG；--show 才让 matplotlib 挑后端。
if "--show" not in sys.argv:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  （必须在选好后端之后再导入 pyplot）

# ---------------------------------------------------------------------------
# 默认读写目录
# ---------------------------------------------------------------------------
# 安装版由 CMake 的 configure_file 把源码树的 draw 目录替换进下面这个占位符；
# 直接在源码树里运行（占位符仍带 @）时，就退回脚本自身所在目录 —— 那正是 draw 目录。
_DRAW_DIR_PLACEHOLDER = "@KDL_IDENTIFY_DRAW_DIR@"


def default_draw_dir():
    """返回默认的读写目录：样本/参数 CSV 与输出图片都放在这里。"""
    if _DRAW_DIR_PLACEHOLDER.startswith("@"):
        return os.path.dirname(os.path.abspath(__file__))
    return _DRAW_DIR_PLACEHOLDER


def parse_args(argv=None):
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="把参数辨识导出的样本/参数 CSV 画成图。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "samples",
        nargs="?",
        default=None,
        help="kdl_identify_node 导出的样本 CSV（缺省：<draw>/identification_samples.csv）",
    )
    parser.add_argument(
        "--params",
        default=None,
        help="参数 CSV（缺省：<draw>/identification_params.csv）",
    )
    parser.add_argument("--out-dir", default=None, help="图片输出目录（缺省：<draw>）")
    parser.add_argument("--show", action="store_true", help="除保存 PNG 外再弹出交互窗口")
    return parser.parse_args(argv)


def load_samples(path):
    """读入样本 CSV，返回结构化数组（表头即字段名）。"""
    data = np.genfromtxt(path, delimiter=",", names=True)
    return np.atleast_1d(data)


def detect_joints(names):
    """按 q0, q1, ... 连续出现来数关节数。"""
    count = 0
    while "q{}".format(count) in names:
        count += 1
    return count


def load_params(path):
    """读参数 CSV，返回 [(index, label, value), ...]。"""
    rows = []
    with open(path, newline="") as handle:
        for row in csv_module.DictReader(handle):
            try:
                rows.append((int(row["index"]), row["label"], float(row["value"])))
            except (KeyError, ValueError):
                continue
    return rows


def plot_joint_curves(ax, t, data, names, fmt, joints, title, ylabel, linestyle="-"):
    """在坐标轴上画每个关节的同名曲线（缺列就跳过）。"""
    drawn = False
    for j in range(joints):
        column = fmt.format(j)
        if column in names:
            ax.plot(t, data[column], linestyle=linestyle, label="J{}".format(j + 1))
            drawn = True
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    return drawn


def build_overview(data, names, joints, residual_columns, source_name):
    """2x3 总览图。"""
    t = np.asarray(data["t"], dtype=float)
    t = t - t[0]  # 平移成从 0 开始，便于看"窗口内"的形状

    fig, axes = plt.subplots(3, 2, figsize=(15, 11))
    flat = axes.ravel()

    plot_joint_curves(flat[0], t, data, names, "q{}", joints,
                      "Joint positions", "q [rad]")
    flat[0].legend(fontsize=7, ncol=2)
    plot_joint_curves(flat[1], t, data, names, "qdot{}", joints,
                      "Joint velocities", "dq/dt [rad/s]")
    plot_joint_curves(flat[2], t, data, names, "qddot{}", joints,
                      "Joint accelerations", "d2q/dt2 [rad/s^2]")

    # 力矩：实测实线、预测虚线。
    has_prediction = any("tau_pred{}".format(j) in names for j in range(joints))
    for j in range(joints):
        measured = "tau{}".format(j)
        if measured in names:
            flat[3].plot(t, data[measured], linestyle="-", linewidth=1.0,
                         label="J{} meas".format(j + 1))
        predicted = "tau_pred{}".format(j)
        if predicted in names:
            flat[3].plot(t, data[predicted], linestyle="--", linewidth=1.0, alpha=0.8,
                         label="J{} pred".format(j + 1))
    flat[3].set_title("Joint torques: measured (solid) vs predicted (dashed)")
    flat[3].set_ylabel("tau [N.m]")
    flat[3].grid(True, alpha=0.3)
    if joints <= 3:
        flat[3].legend(fontsize=7, ncol=2)

    # 残差：tau_meas - tau_pred。
    if has_prediction:
        for j in range(joints):
            measured = "tau{}".format(j)
            predicted = "tau_pred{}".format(j)
            if measured in names and predicted in names:
                flat[4].plot(t, data[measured] - data[predicted], linewidth=1.0,
                             label="J{}".format(j + 1))
        flat[4].set_title("Torque residual: measured - predicted")
        flat[4].set_ylabel("residual [N.m]")
        flat[4].grid(True, alpha=0.3)
        flat[4].legend(fontsize=7, ncol=2)
    else:
        flat[4].axis("off")
        flat[4].text(0.0, 0.5, "no tau_pred column:\nre-run kdl_identify_node to get it",
                     va="center", ha="left", family="monospace", fontsize=10)

    for ax in (flat[0], flat[1], flat[2], flat[3], flat[4]):
        ax.set_xlabel("t - t0 [s]")

    # 信息格。
    flat[5].axis("off")
    lines = [
        "samples : {}".format(t.size),
        "span    : {:.4f} s".format(float(t[-1] - t[0])),
        "dt      : {:.5f} s".format(float(np.mean(np.diff(t))) if t.size > 1 else 0.0),
        "joints  : {}".format(joints),
    ]
    if has_prediction:
        all_residual = np.concatenate([residual_columns[j] for j in range(joints)
                                       if j in residual_columns]) if residual_columns else None
        if all_residual is not None and all_residual.size > 0:
            lines += [
                "",
                "residual RMS : {:.3e} N.m".format(
                    float(np.sqrt(np.mean(all_residual ** 2)))),
                "residual MAX : {:.3e} N.m".format(float(np.max(np.abs(all_residual)))),
                "",
            ]
        for j in range(joints):
            if j in residual_columns:
                values = residual_columns[j]
                lines.append("  J{} rms = {:.3e}".format(
                    j + 1, float(np.sqrt(np.mean(values ** 2)))))
    flat[5].text(0.0, 1.0, "\n".join(lines), va="top", ha="left",
                 family="monospace", fontsize=10)

    fig.suptitle("Robot parameter identification - {}".format(source_name))
    fig.tight_layout()
    return fig


def build_joint_figure(data, names, joint):
    """单个关节的力矩对比 + 残差。"""
    t = np.asarray(data["t"], dtype=float)
    t = t - t[0]
    measured = "tau{}".format(joint)
    predicted = "tau_pred{}".format(joint)

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    if measured in names:
        axes[0].plot(t, data[measured], label="measured")
    if predicted in names:
        axes[0].plot(t, data[predicted], linestyle="--", label="predicted")
    axes[0].set_title("Joint {} torque: measured vs predicted".format(joint + 1))
    axes[0].set_ylabel("tau [N.m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=9)

    if measured in names and predicted in names:
        axes[1].plot(t, data[measured] - data[predicted], color="crimson", linewidth=1.0)
        axes[1].set_title("Residual (measured - predicted)")
        axes[1].set_ylabel("residual [N.m]")
    axes[1].set_xlabel("t - t0 [s]")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def build_parameters_figure(params):
    """基参数（含摩擦项）水平条形图。"""
    rows = sorted(params, key=lambda item: item[0])
    labels = [row[1] for row in rows]
    values = [row[2] for row in rows]

    colors = []
    for label in labels:
        if label.endswith("viscous_friction"):
            colors.append("darkorange")
        elif label.endswith("coulomb_friction"):
            colors.append("goldenrod")
        else:
            colors.append("steelblue")

    height = max(4.0, 0.22 * len(rows) + 1.5)
    fig, ax = plt.subplots(figsize=(10, height))
    y = np.arange(len(rows))
    ax.barh(y, values, color=colors)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=7)
    ax.invert_yaxis()  # 第一个参数画在最上面
    ax.axvline(0.0, color="black", linewidth=0.8)
    ax.set_xlabel("identified value (unit depends on parameter)")
    ax.set_title("Identified base parameters ({} total)".format(len(rows)))
    ax.grid(True, axis="x", alpha=0.3)
    fig.tight_layout()
    return fig


def build_friction_figure(params):
    """各关节粘性/库伦摩擦对照（没有摩擦项时返回 None）。"""
    viscous = {}
    coulomb = {}
    for _, label, value in params:
        if ".viscous_friction" in label:
            viscous[label.split(".")[0]] = value
        elif ".coulomb_friction" in label:
            coulomb[label.split(".")[0]] = value
    if not viscous and not coulomb:
        return None

    joints = sorted(set(list(viscous.keys()) + list(coulomb.keys())),
                    key=lambda name: int("".join(ch for ch in name if ch.isdigit()) or 0))
    index = np.arange(len(joints))
    width = 0.38

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.bar(index - width / 2, [viscous.get(j, 0.0) for j in joints], width,
           label="viscous", color="darkorange")
    ax.bar(index + width / 2, [coulomb.get(j, 0.0) for j in joints], width,
           label="coulomb", color="goldenrod")
    ax.set_xticks(index)
    ax.set_xticklabels(joints)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_ylabel("friction coefficient")
    ax.set_title("Identified joint friction")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    return fig


def main(argv=None):
    """读 CSV、画图、保存，并按需弹窗。"""
    args = parse_args(argv)

    draw_dir = default_draw_dir()
    samples_path = args.samples or os.path.join(draw_dir, "identification_samples.csv")

    if not os.path.isfile(samples_path):
        print("找不到样本 CSV: {}".format(samples_path))
        print("提示: 先跑 `ros2 launch kdl_tools identify.launch.py headless:=true`"
              "（CSV 会落在 draw 目录里）。")
        return 1

    data = load_samples(samples_path)
    names = set(data.dtype.names or ())
    if "t" not in names:
        print("样本 CSV 里没有 t 列，不像本包导出的文件: {}".format(samples_path))
        return 1

    joints = detect_joints(names)
    if joints == 0:
        print("样本 CSV 里找不到 q0 这种列: {}".format(samples_path))
        return 1

    params_path = args.params or os.path.join(draw_dir, "identification_params.csv")

    out_dir = args.out_dir or draw_dir
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(samples_path))[0]

    # 预先把每个关节的残差算好，总览的信息格与分关节图都要用。
    residual_columns = {}
    for j in range(joints):
        measured = "tau{}".format(j)
        predicted = "tau_pred{}".format(j)
        if measured in names and predicted in names:
            residual_columns[j] = np.asarray(data[measured] - data[predicted], dtype=float)

    written = []
    figures = []

    overview = build_overview(data, names, joints, residual_columns,
                              os.path.basename(samples_path))
    path = os.path.join(out_dir, "{}_overview.png".format(stem))
    overview.savefig(path, bbox_inches="tight")
    figures.append(overview)
    written.append(path)

    for j in range(joints):
        figure = build_joint_figure(data, names, j)
        path = os.path.join(out_dir, "{}_torque_joint{}.png".format(stem, j + 1))
        figure.savefig(path, bbox_inches="tight")
        figures.append(figure)
        written.append(path)

    params = []
    if os.path.isfile(params_path):
        params = load_params(params_path)
        if params:
            figure = build_parameters_figure(params)
            path = os.path.join(out_dir, "{}_parameters.png".format(stem))
            figure.savefig(path, bbox_inches="tight")
            figures.append(figure)
            written.append(path)

            friction = build_friction_figure(params)
            if friction is not None:
                path = os.path.join(out_dir, "{}_friction.png".format(stem))
                friction.savefig(path, bbox_inches="tight")
                figures.append(friction)
                written.append(path)
    else:
        print("没找到参数 CSV: {}（跳过参数图）".format(params_path))

    print("已读取 {} 个采样点、{} 个关节:".format(len(data["t"]), joints))
    if residual_columns:
        all_values = np.concatenate([residual_columns[j] for j in sorted(residual_columns)])
        print("  残差 RMS = {:.3e} N.m，MAX = {:.3e} N.m".format(
            float(np.sqrt(np.mean(all_values ** 2))), float(np.max(np.abs(all_values)))))
    for path in written:
        print("  {}".format(os.path.abspath(path)))

    if args.show:
        plt.show()

    for figure in figures:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    sys.exit(main())
