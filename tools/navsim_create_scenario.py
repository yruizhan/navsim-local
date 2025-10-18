#!/usr/bin/env python3
"""
NavSim Scenario Creation Tool

交互式创建和编辑 JSON 场景文件。

用法:
    # 交互式创建场景
    python3 tools/navsim_create_scenario.py --output scenarios/my_scenario.json
    
    # 从模板创建场景
    python3 tools/navsim_create_scenario.py \\
        --template corridor \\
        --output scenarios/my_corridor.json
    
    # 编辑现有场景
    python3 tools/navsim_create_scenario.py \\
        --edit scenarios/simple_corridor.json \\
        --output scenarios/modified_corridor.json
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional


# ========== 场景模板 ==========

TEMPLATES = {
    "empty": {
        "name": "Empty Scenario",
        "description": "Empty scenario template",
        "ego_vehicle": {
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
            "kinematics": {
                "wheelbase": 2.8,
                "width": 2.0,
                "length": 4.8
            },
            "limits": {
                "max_velocity": 15.0,
                "max_acceleration": 3.0,
                "max_steer_angle": 0.6
            }
        },
        "task": {
            "goal_pose": {"x": 10.0, "y": 0.0, "yaw": 0.0},
            "type": "GOTO_GOAL",
            "tolerance": {"position": 0.5, "yaw": 0.2}
        },
        "static_obstacles": [],
        "dynamic_obstacles": []
    },
    
    "corridor": {
        "name": "Corridor Scenario",
        "description": "Navigate through a corridor with obstacles",
        "ego_vehicle": {
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
            "kinematics": {
                "wheelbase": 2.8,
                "width": 2.0,
                "length": 4.8
            },
            "limits": {
                "max_velocity": 15.0,
                "max_acceleration": 3.0,
                "max_steer_angle": 0.6
            }
        },
        "task": {
            "goal_pose": {"x": 20.0, "y": 0.0, "yaw": 0.0},
            "type": "GOTO_GOAL",
            "tolerance": {"position": 0.5, "yaw": 0.2}
        },
        "static_obstacles": [
            {
                "type": "rectangle",
                "pose": {"x": 10.0, "y": 3.0, "yaw": 0.0},
                "width": 2.0,
                "height": 5.0
            },
            {
                "type": "rectangle",
                "pose": {"x": 10.0, "y": -3.0, "yaw": 0.0},
                "width": 2.0,
                "height": 5.0
            }
        ],
        "dynamic_obstacles": []
    },
    
    "parking": {
        "name": "Parking Scenario",
        "description": "Park in a tight space",
        "ego_vehicle": {
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
            "kinematics": {
                "wheelbase": 2.8,
                "width": 2.0,
                "length": 4.8
            },
            "limits": {
                "max_velocity": 5.0,
                "max_acceleration": 2.0,
                "max_steer_angle": 0.6
            }
        },
        "task": {
            "goal_pose": {"x": 15.0, "y": 3.0, "yaw": 1.57},
            "type": "PARKING",
            "tolerance": {"position": 0.2, "yaw": 0.1}
        },
        "static_obstacles": [
            {
                "type": "rectangle",
                "pose": {"x": 15.0, "y": 0.0, "yaw": 1.57},
                "width": 2.0,
                "height": 5.0
            },
            {
                "type": "rectangle",
                "pose": {"x": 15.0, "y": 6.0, "yaw": 1.57},
                "width": 2.0,
                "height": 5.0
            }
        ],
        "dynamic_obstacles": []
    }
}


# ========== 交互式输入函数 ==========

def input_float(prompt: str, default: float) -> float:
    """输入浮点数"""
    while True:
        value = input(f"{prompt} [{default}]: ").strip()
        if not value:
            return default
        try:
            return float(value)
        except ValueError:
            print("  Invalid number, please try again.")


def input_int(prompt: str, default: int) -> int:
    """输入整数"""
    while True:
        value = input(f"{prompt} [{default}]: ").strip()
        if not value:
            return default
        try:
            return int(value)
        except ValueError:
            print("  Invalid integer, please try again.")


def input_bool(prompt: str, default: bool) -> bool:
    """输入布尔值"""
    default_str = "y" if default else "n"
    while True:
        value = input(f"{prompt} (y/n) [{default_str}]: ").strip().lower()
        if not value:
            return default
        if value in ['y', 'yes', 'true', '1']:
            return True
        if value in ['n', 'no', 'false', '0']:
            return False
        print("  Invalid input, please enter y or n.")


def input_pose(prompt: str, default: Dict[str, float]) -> Dict[str, float]:
    """输入位姿"""
    print(f"\n{prompt}")
    return {
        "x": input_float("  x (m)", default.get("x", 0.0)),
        "y": input_float("  y (m)", default.get("y", 0.0)),
        "yaw": input_float("  yaw (rad)", default.get("yaw", 0.0))
    }


def input_obstacle() -> Optional[Dict[str, Any]]:
    """输入障碍物"""
    print("\nAdd obstacle:")
    print("  1. Circle")
    print("  2. Rectangle")
    print("  3. Skip")
    
    choice = input("Choose obstacle type [3]: ").strip()
    
    if choice == "1":
        return {
            "type": "circle",
            "center": {
                "x": input_float("  Center x (m)", 0.0),
                "y": input_float("  Center y (m)", 0.0)
            },
            "radius": input_float("  Radius (m)", 1.0)
        }
    elif choice == "2":
        return {
            "type": "rectangle",
            "pose": input_pose("  Rectangle pose:", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
            "width": input_float("  Width (m)", 2.0),
            "height": input_float("  Height (m)", 5.0)
        }
    else:
        return None


# ========== 场景创建函数 ==========

def create_scenario_interactive() -> Dict[str, Any]:
    """交互式创建场景"""
    print("\n=== NavSim Scenario Creator ===\n")
    
    scenario = {}
    
    # 基本信息
    scenario["name"] = input("Scenario name: ").strip() or "Unnamed Scenario"
    scenario["description"] = input("Description: ").strip() or ""
    
    # 自车状态
    print("\n--- Ego Vehicle ---")
    scenario["ego_vehicle"] = {
        "pose": input_pose("Initial pose:", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
        "twist": {
            "vx": input_float("Initial velocity vx (m/s)", 0.0),
            "vy": input_float("Initial velocity vy (m/s)", 0.0),
            "omega": input_float("Initial angular velocity (rad/s)", 0.0)
        },
        "kinematics": {
            "wheelbase": input_float("Wheelbase (m)", 2.8),
            "width": input_float("Width (m)", 2.0),
            "length": input_float("Length (m)", 4.8)
        },
        "limits": {
            "max_velocity": input_float("Max velocity (m/s)", 15.0),
            "max_acceleration": input_float("Max acceleration (m/s²)", 3.0),
            "max_steer_angle": input_float("Max steer angle (rad)", 0.6)
        }
    }
    
    # 任务目标
    print("\n--- Task ---")
    scenario["task"] = {
        "goal_pose": input_pose("Goal pose:", {"x": 10.0, "y": 0.0, "yaw": 0.0}),
        "type": "GOTO_GOAL",
        "tolerance": {
            "position": input_float("Position tolerance (m)", 0.5),
            "yaw": input_float("Yaw tolerance (rad)", 0.2)
        }
    }
    
    # 静态障碍物
    print("\n--- Static Obstacles ---")
    scenario["static_obstacles"] = []
    while True:
        obstacle = input_obstacle()
        if obstacle is None:
            break
        scenario["static_obstacles"].append(obstacle)
        if not input_bool("Add another obstacle?", False):
            break
    
    # 动态障碍物
    scenario["dynamic_obstacles"] = []
    
    return scenario


def create_scenario_from_template(template_name: str) -> Dict[str, Any]:
    """从模板创建场景"""
    if template_name not in TEMPLATES:
        raise ValueError(f"Unknown template: {template_name}")
    
    return TEMPLATES[template_name].copy()


def save_scenario(scenario: Dict[str, Any], output_path: Path):
    """保存场景到文件"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(scenario, f, indent=2, ensure_ascii=False)
    
    print(f"\n✅ Scenario saved to: {output_path}")


def load_scenario(input_path: Path) -> Dict[str, Any]:
    """从文件加载场景"""
    with open(input_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser(
        description="NavSim Scenario Creation Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        "--output",
        required=True,
        help="Output scenario file path"
    )
    
    parser.add_argument(
        "--template",
        choices=list(TEMPLATES.keys()),
        help="Use a template (empty, corridor, parking)"
    )
    
    parser.add_argument(
        "--edit",
        help="Edit an existing scenario file"
    )
    
    parser.add_argument(
        "--list-templates",
        action="store_true",
        help="List available templates"
    )
    
    args = parser.parse_args()
    
    # 列出模板
    if args.list_templates:
        print("\nAvailable templates:")
        for name, template in TEMPLATES.items():
            print(f"  - {name}: {template['description']}")
        return 0
    
    try:
        # 创建场景
        if args.edit:
            # 编辑现有场景
            scenario = load_scenario(Path(args.edit))
            print(f"Loaded scenario from: {args.edit}")
            # TODO: 实现编辑功能
        elif args.template:
            # 从模板创建
            scenario = create_scenario_from_template(args.template)
            print(f"Created scenario from template: {args.template}")
        else:
            # 交互式创建
            scenario = create_scenario_interactive()
        
        # 保存场景
        save_scenario(scenario, Path(args.output))
        
        return 0
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

