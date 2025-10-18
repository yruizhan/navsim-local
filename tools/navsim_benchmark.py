#!/usr/bin/env python3
"""
NavSim Benchmark Tool

批量测试场景，生成性能报告。

用法:
    # 测试单个规划器
    python3 tools/navsim_benchmark.py \\
        --planner JpsPlanner \\
        --scenarios scenarios/*.json \\
        --output reports/jps_benchmark.json
    
    # 对比多个规划器
    python3 tools/navsim_benchmark.py \\
        --planners JpsPlanner,AStarPlanner,StraightLinePlanner \\
        --scenarios scenarios/*.json \\
        --output reports/comparison.json \\
        --html reports/comparison.html
    
    # 指定感知插件
    python3 tools/navsim_benchmark.py \\
        --planner JpsPlanner \\
        --perception GridMapBuilder,ESDFBuilder \\
        --scenarios scenarios/*.json
"""

import argparse
import glob
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, asdict
import statistics


@dataclass
class BenchmarkResult:
    """单次测试结果"""
    scenario: str
    planner: str
    perception: List[str]
    success: bool
    trajectory_points: int
    computation_time_ms: float
    failure_reason: str = ""


@dataclass
class BenchmarkSummary:
    """测试汇总"""
    planner: str
    total_scenarios: int
    success_count: int
    failure_count: int
    success_rate: float
    avg_computation_time_ms: float
    min_computation_time_ms: float
    max_computation_time_ms: float
    std_computation_time_ms: float


def run_single_test(
    navsim_debug_path: Path,
    scenario_path: Path,
    planner: str,
    perception: List[str],
    verbose: bool = False
) -> BenchmarkResult:
    """运行单次测试"""
    
    # 构建命令
    cmd = [
        str(navsim_debug_path),
        "--scenario", str(scenario_path),
        "--planner", planner
    ]
    
    if perception:
        cmd.extend(["--perception", ",".join(perception)])
    
    if verbose:
        print(f"  Running: {' '.join(cmd)}")
    
    # 运行测试
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=30
        )
        
        output = result.stdout
        
        # 解析输出
        success = "Success: yes" in output
        trajectory_points = 0
        computation_time_ms = 0.0
        failure_reason = ""
        
        for line in output.split('\n'):
            if "Trajectory points:" in line:
                try:
                    trajectory_points = int(line.split(':')[1].strip())
                except:
                    pass
            elif "Computation time:" in line:
                try:
                    time_str = line.split(':')[1].strip().split()[0]
                    computation_time_ms = float(time_str)
                except:
                    pass
            elif "Failure reason:" in line:
                failure_reason = line.split(':', 1)[1].strip()
        
        return BenchmarkResult(
            scenario=scenario_path.name,
            planner=planner,
            perception=perception,
            success=success,
            trajectory_points=trajectory_points,
            computation_time_ms=computation_time_ms,
            failure_reason=failure_reason
        )
        
    except subprocess.TimeoutExpired:
        return BenchmarkResult(
            scenario=scenario_path.name,
            planner=planner,
            perception=perception,
            success=False,
            trajectory_points=0,
            computation_time_ms=0.0,
            failure_reason="Timeout (30s)"
        )
    except Exception as e:
        return BenchmarkResult(
            scenario=scenario_path.name,
            planner=planner,
            perception=perception,
            success=False,
            trajectory_points=0,
            computation_time_ms=0.0,
            failure_reason=str(e)
        )


def compute_summary(results: List[BenchmarkResult], planner: str) -> BenchmarkSummary:
    """计算汇总统计"""
    
    planner_results = [r for r in results if r.planner == planner]
    
    if not planner_results:
        return BenchmarkSummary(
            planner=planner,
            total_scenarios=0,
            success_count=0,
            failure_count=0,
            success_rate=0.0,
            avg_computation_time_ms=0.0,
            min_computation_time_ms=0.0,
            max_computation_time_ms=0.0,
            std_computation_time_ms=0.0
        )
    
    success_count = sum(1 for r in planner_results if r.success)
    failure_count = len(planner_results) - success_count
    success_rate = success_count / len(planner_results) * 100
    
    computation_times = [r.computation_time_ms for r in planner_results if r.success]
    
    if computation_times:
        avg_time = statistics.mean(computation_times)
        min_time = min(computation_times)
        max_time = max(computation_times)
        std_time = statistics.stdev(computation_times) if len(computation_times) > 1 else 0.0
    else:
        avg_time = min_time = max_time = std_time = 0.0
    
    return BenchmarkSummary(
        planner=planner,
        total_scenarios=len(planner_results),
        success_count=success_count,
        failure_count=failure_count,
        success_rate=success_rate,
        avg_computation_time_ms=avg_time,
        min_computation_time_ms=min_time,
        max_computation_time_ms=max_time,
        std_computation_time_ms=std_time
    )


def print_summary(summary: BenchmarkSummary):
    """打印汇总信息"""
    print(f"\n{'='*60}")
    print(f"Planner: {summary.planner}")
    print(f"{'='*60}")
    print(f"Total scenarios:      {summary.total_scenarios}")
    print(f"Success:              {summary.success_count} ({summary.success_rate:.1f}%)")
    print(f"Failure:              {summary.failure_count}")
    print(f"Avg computation time: {summary.avg_computation_time_ms:.2f} ms")
    print(f"Min computation time: {summary.min_computation_time_ms:.2f} ms")
    print(f"Max computation time: {summary.max_computation_time_ms:.2f} ms")
    print(f"Std computation time: {summary.std_computation_time_ms:.2f} ms")


def save_json_report(results: List[BenchmarkResult], summaries: List[BenchmarkSummary], output_path: Path):
    """保存 JSON 报告"""
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "results": [asdict(r) for r in results],
        "summaries": [asdict(s) for s in summaries]
    }
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    
    print(f"\n✅ JSON report saved to: {output_path}")


def save_html_report(results: List[BenchmarkResult], summaries: List[BenchmarkSummary], output_path: Path):
    """保存 HTML 报告"""
    
    html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>NavSim Benchmark Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        h1 {{ color: #333; }}
        table {{ border-collapse: collapse; width: 100%; margin: 20px 0; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #4CAF50; color: white; }}
        tr:nth-child(even) {{ background-color: #f2f2f2; }}
        .success {{ color: green; }}
        .failure {{ color: red; }}
    </style>
</head>
<body>
    <h1>NavSim Benchmark Report</h1>
    <p>Generated: {time.strftime("%Y-%m-%d %H:%M:%S")}</p>
    
    <h2>Summary</h2>
    <table>
        <tr>
            <th>Planner</th>
            <th>Total</th>
            <th>Success</th>
            <th>Failure</th>
            <th>Success Rate</th>
            <th>Avg Time (ms)</th>
        </tr>
"""
    
    for summary in summaries:
        html += f"""        <tr>
            <td>{summary.planner}</td>
            <td>{summary.total_scenarios}</td>
            <td class="success">{summary.success_count}</td>
            <td class="failure">{summary.failure_count}</td>
            <td>{summary.success_rate:.1f}%</td>
            <td>{summary.avg_computation_time_ms:.2f}</td>
        </tr>
"""
    
    html += """    </table>
    
    <h2>Detailed Results</h2>
    <table>
        <tr>
            <th>Scenario</th>
            <th>Planner</th>
            <th>Success</th>
            <th>Points</th>
            <th>Time (ms)</th>
            <th>Failure Reason</th>
        </tr>
"""
    
    for result in results:
        success_class = "success" if result.success else "failure"
        success_text = "✓" if result.success else "✗"
        html += f"""        <tr>
            <td>{result.scenario}</td>
            <td>{result.planner}</td>
            <td class="{success_class}">{success_text}</td>
            <td>{result.trajectory_points}</td>
            <td>{result.computation_time_ms:.2f}</td>
            <td>{result.failure_reason}</td>
        </tr>
"""
    
    html += """    </table>
</body>
</html>
"""
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)
    
    print(f"✅ HTML report saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="NavSim Benchmark Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        "--planner",
        help="Single planner to test"
    )
    
    parser.add_argument(
        "--planners",
        help="Multiple planners to test (comma-separated)"
    )
    
    parser.add_argument(
        "--perception",
        default="",
        help="Perception plugins (comma-separated)"
    )
    
    parser.add_argument(
        "--scenarios",
        nargs='+',
        required=True,
        help="Scenario files (supports wildcards)"
    )
    
    parser.add_argument(
        "--navsim-debug",
        default="build/navsim_local_debug",
        help="Path to navsim_local_debug executable"
    )
    
    parser.add_argument(
        "--output",
        help="Output JSON report path"
    )
    
    parser.add_argument(
        "--html",
        help="Output HTML report path"
    )
    
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Verbose output"
    )
    
    args = parser.parse_args()
    
    # 解析规划器列表
    if args.planners:
        planners = [p.strip() for p in args.planners.split(',')]
    elif args.planner:
        planners = [args.planner]
    else:
        print("Error: Must specify --planner or --planners")
        return 1
    
    # 解析感知插件
    perception = [p.strip() for p in args.perception.split(',') if p.strip()]
    
    # 展开场景文件通配符
    scenario_files = []
    for pattern in args.scenarios:
        scenario_files.extend(glob.glob(pattern))
    
    scenario_files = [Path(f) for f in scenario_files if Path(f).exists()]
    
    if not scenario_files:
        print("Error: No scenario files found")
        return 1
    
    # 检查 navsim_debug 可执行文件
    navsim_debug_path = Path(args.navsim_debug)
    if not navsim_debug_path.exists():
        print(f"Error: navsim_local_debug not found: {navsim_debug_path}")
        return 1
    
    print(f"\n=== NavSim Benchmark ===")
    print(f"Planners: {', '.join(planners)}")
    print(f"Perception: {', '.join(perception) if perception else 'None'}")
    print(f"Scenarios: {len(scenario_files)}")
    print()
    
    # 运行测试
    all_results = []
    
    for planner in planners:
        print(f"\nTesting {planner}...")
        
        for scenario_file in scenario_files:
            print(f"  {scenario_file.name}...", end=' ', flush=True)
            
            result = run_single_test(
                navsim_debug_path,
                scenario_file,
                planner,
                perception,
                args.verbose
            )
            
            all_results.append(result)
            
            if result.success:
                print(f"✓ ({result.computation_time_ms:.2f} ms)")
            else:
                print(f"✗ ({result.failure_reason})")
    
    # 计算汇总
    summaries = [compute_summary(all_results, planner) for planner in planners]
    
    # 打印汇总
    for summary in summaries:
        print_summary(summary)
    
    # 保存报告
    if args.output:
        save_json_report(all_results, summaries, Path(args.output))
    
    if args.html:
        save_html_report(all_results, summaries, Path(args.html))
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

