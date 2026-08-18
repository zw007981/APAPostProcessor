#!/usr/bin/env python3
"""算法性能 profiling 一条龙工具：配置 Profile 版 -> perf 采样 -> 生成火焰图。

用法：
    python3 tool/profile.py              # 直接跑，用 DEFAULT_ALGORITHM / DEFAULT_REPEATS
    python3 tool/profile.py alm          # 指定算法（重复次数仍取 DEFAULT_REPEATS）
    python3 tool/profile.py ilqr --repeats 3
    python3 tool/profile.py nmpc

算法由 ALGORITHMS 注册表驱动：新增算法只需追加一条记录（键名 -> 配置路径、
火焰图标题、产物文件名），并准备对应的 data/<算法>_config.json（其中
"algorithm" 字段决定场景工厂路由），无需改动其余代码。
"""

import argparse
import glob
import os
import subprocess
from dataclasses import dataclass
from typing import Dict, List

# 项目根目录（本文件位于 tool/，故上溯一级）
PROJECT_ROOT: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Profile 构建目录（RelWithDebInfo + 保留帧指针）
BUILD_DIR: str = os.path.join(PROJECT_ROOT, "build", "Profile")
# 火焰图产物目录
FIG_DIR: str = os.path.join(PROJECT_ROOT, "fig")
# 通用 profiling 驱动的 CMake 目标名与二进制名（同名）
PROFILE_TARGET: str = "apa_profile"
# 直接运行本文件（不带命令行参数）时 profile 的算法；命令行传入算法可覆盖
DEFAULT_ALGORITHM: str = "alm"
# 直接运行本文件时每数据集的重复次数；--repeats 参数可覆盖
DEFAULT_REPEATS: int = 5


@dataclass(frozen=True)
class ProfileSpec:
    """单个算法的 profiling 参数集合"""

    config: str
    """算法配置详情 JSON（相对项目根，决定场景工厂的算法路由）"""
    title: str
    """火焰图标题"""
    svg_name: str
    """火焰图产物文件名（落盘 fig/ 目录）"""
    perf_name: str
    """perf.data 文件名（落盘 build/Profile）"""


# 算法注册表：新增算法在此追加一条记录即可
ALGORITHMS: Dict[str, ProfileSpec] = {
    "alm": ProfileSpec(
        "data/alm_config.json",
        "ALM 4-dataset profile",
        "alm_flame.svg",
        "perf_alm.data",
    ),
    "ilqr": ProfileSpec(
        "data/ilqr_config.json",
        "iLQR 4-dataset profile",
        "ilqr_flame.svg",
        "perf_ilqr.data",
    ),
    "nmpc": ProfileSpec(
        "data/nmpc_config.json",
        "NMPC 4-dataset profile",
        "nmpc_flame.svg",
        "perf_nmpc.data",
    ),
}


def findPerfBin() -> str:
    """定位 WSL2 下 perf 的真实二进制，缺失时抛异常"""
    matches: List[str] = sorted(glob.glob("/usr/lib/linux-tools-*/perf"))
    if not matches:
        raise FileNotFoundError(
            "未找到 perf 真实二进制，请 sudo apt install linux-tools-generic"
        )
    return matches[0]


def findFlameGraphDir() -> str:
    """定位 Brendan Gregg FlameGraph 目录，校验 flamegraph.pl 存在"""
    flame_dir: str = os.environ.get(
        "FLAMEGRAPH_DIR", os.path.expanduser("~/tools/FlameGraph")
    )
    if not os.path.isfile(os.path.join(flame_dir, "flamegraph.pl")):
        raise FileNotFoundError(
            f"未找到 {flame_dir}/flamegraph.pl，"
            "请 git clone https://github.com/brendangregg/FlameGraph"
        )
    return flame_dir


def configureProject() -> None:
    """配置 Profile 构建目录（RelWithDebInfo + 保留帧指针，幂等）"""
    subprocess.run(
        [
            "cmake",
            "-S",
            PROJECT_ROOT,
            "-B",
            BUILD_DIR,
            "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
            "-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer",
        ],
        check=True,
    )


def buildTarget() -> None:
    """增量构建通用 profiling 驱动"""
    subprocess.run(
        ["cmake", "--build", BUILD_DIR, "--target", PROFILE_TARGET, "-j"],
        check=True,
    )


def recordPerf(perf_bin: str, spec: ProfileSpec, repeats: int) -> str:
    """perf 采样并返回 perf.data 路径；固定 4 线程、关闭自旋避免污染"""
    perf_data_path: str = os.path.join(BUILD_DIR, spec.perf_name)
    binary_path: str = os.path.join(BUILD_DIR, PROFILE_TARGET)
    env: Dict[str, str] = os.environ.copy()
    # 与生产基线一致固定线程数；GOMP_SPINCOUNT=0 让串行求解期间的工作线程
    # 立即睡眠而非忙等自旋（否则 libgomp.so 自旋帧会淹没真实热点）
    env["OMP_NUM_THREADS"] = "4"
    env["GOMP_SPINCOUNT"] = "0"
    subprocess.run(
        [
            perf_bin,
            "record",
            "-e",
            "cpu-clock",
            "-F",
            "999",
            "--call-graph",
            "fp",
            "-o",
            perf_data_path,
            "--",
            binary_path,
            spec.config,
            str(repeats),
        ],
        cwd=PROJECT_ROOT,
        env=env,
        check=True,
    )
    return perf_data_path


def renderFlameGraph(
    perf_bin: str, flame_dir: str, spec: ProfileSpec, perf_data_path: str
) -> None:
    """折叠调用栈并渲染火焰图：perf script | stackcollapse | flamegraph"""
    os.makedirs(FIG_DIR, exist_ok=True)
    svg_path: str = os.path.join(FIG_DIR, spec.svg_name)
    collapse_script: str = os.path.join(flame_dir, "stackcollapse-perf.pl")
    flamegraph_script: str = os.path.join(flame_dir, "flamegraph.pl")
    # 用 bash -o pipefail 保证任一段失败都能以非零码退出
    command: str = (
        f'set -o pipefail; "{perf_bin}" script -i "{perf_data_path}" | '
        f'"{collapse_script}" | "{flamegraph_script}" '
        f'--title "{spec.title}" > "{svg_path}"'
    )
    subprocess.run(["bash", "-c", command], check=True)
    print(f"✓ 完成：{svg_path}")


def profileAlgorithm(spec: ProfileSpec, repeats: int) -> None:
    """执行完整 profiling 流水线：配置 -> 构建 -> 采样 -> 火焰图"""
    perf_bin: str = findPerfBin()
    flame_dir: str = findFlameGraphDir()
    configureProject()
    buildTarget()
    print(
        f"==> perf 采样（cpu-clock @999Hz，每数据集重复 {repeats} 次，"
        f"配置 {spec.config}）"
    )
    perf_data_path: str = recordPerf(perf_bin, spec, repeats)
    print(f"==> 生成火焰图 fig/{spec.svg_name}")
    renderFlameGraph(perf_bin, flame_dir, spec, perf_data_path)


def main() -> None:
    """解析命令行参数并调度对应算法的 profiling 流水线"""
    parser = argparse.ArgumentParser(description="算法性能 profiling 一条龙工具")
    parser.add_argument(
        "algorithm",
        nargs="?",
        choices=sorted(ALGORITHMS.keys()),
        default=None,
        help="要 profile 的算法（缺省取 DEFAULT_ALGORITHM）",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=None,
        help="每数据集重复次数（缺省取 DEFAULT_REPEATS）",
    )
    args = parser.parse_args()
    algorithm: str = args.algorithm if args.algorithm is not None else DEFAULT_ALGORITHM
    spec: ProfileSpec = ALGORITHMS[algorithm]
    repeats: int = args.repeats if args.repeats is not None else DEFAULT_REPEATS
    if repeats < 1:
        raise ValueError("--repeats 必须为正整数")
    profileAlgorithm(spec, repeats)


if __name__ == "__main__":
    main()
