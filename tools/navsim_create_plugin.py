#!/usr/bin/env python3
"""
NavSim Plugin Scaffolding Tool

自动生成插件模板代码，包括：
- algorithm/ 层（纯算法实现）
- adapter/ 层（平台接口适配）
- CMakeLists.txt
- README.md
- 测试文件（可选）

用法:
    python3 tools/navsim_create_plugin.py \
        --name MyPlanner \
        --type planner \
        --output plugins/planning/my_planner \
        --author "Your Name" \
        --description "My awesome planner"
"""

import argparse
import os
import re
import shutil
from pathlib import Path
from typing import Dict


def to_snake_case(name: str) -> str:
    """
    将 CamelCase 转换为 snake_case
    
    Examples:
        MyPlanner -> my_planner
        AStarPlanner -> a_star_planner
        ESDFBuilder -> esdf_builder
    """
    # 处理连续大写字母
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    # 处理大写字母后跟小写字母的情况
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1)
    return s2.lower()


def get_namespace(plugin_name: str) -> str:
    """
    生成命名空间
    
    Examples:
        MyPlanner -> my_planner
    """
    return to_snake_case(plugin_name)


def replace_template_variables(content: str, variables: Dict[str, str]) -> str:
    """
    替换模板变量
    
    变量格式: {{VARIABLE_NAME}}
    """
    for key, value in variables.items():
        placeholder = f"{{{{{key}}}}}"
        content = content.replace(placeholder, value)
    return content


def copy_template(
    template_dir: Path,
    output_dir: Path,
    variables: Dict[str, str],
    verbose: bool = False
):
    """
    复制模板并替换变量
    """
    if not template_dir.exists():
        raise FileNotFoundError(f"Template directory not found: {template_dir}")
    
    # 创建输出目录
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 遍历模板目录
    for item in template_dir.rglob('*'):
        if item.is_file():
            # 计算相对路径
            rel_path = item.relative_to(template_dir)
            
            # 替换文件名中的变量
            output_path_str = str(rel_path)
            for key, value in variables.items():
                placeholder = f"{{{{{key}}}}}"
                output_path_str = output_path_str.replace(placeholder, value)
            
            output_path = output_dir / output_path_str
            
            # 创建父目录
            output_path.parent.mkdir(parents=True, exist_ok=True)
            
            # 读取文件内容
            try:
                with open(item, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # 替换内容中的变量
                content = replace_template_variables(content, variables)
                
                # 写入输出文件
                with open(output_path, 'w', encoding='utf-8') as f:
                    f.write(content)
                
                if verbose:
                    print(f"  Created: {output_path}")
                    
            except UnicodeDecodeError:
                # 二进制文件直接复制
                shutil.copy2(item, output_path)
                if verbose:
                    print(f"  Copied: {output_path}")


def create_plugin(
    plugin_name: str,
    plugin_type: str,
    output_dir: str,
    author: str = "NavSim Developer",
    description: str = "",
    verbose: bool = False
):
    """
    创建插件
    """
    # 获取脚本所在目录
    script_dir = Path(__file__).parent.parent
    
    # 模板目录
    template_dir = script_dir / "templates" / f"{plugin_type}_plugin"
    
    if not template_dir.exists():
        raise ValueError(f"Unknown plugin type: {plugin_type}")
    
    # 输出目录
    output_path = Path(output_dir)
    
    # 准备模板变量
    plugin_name_snake = to_snake_case(plugin_name)
    namespace = get_namespace(plugin_name)
    
    if not description:
        description = f"{plugin_name} {plugin_type} plugin"
    
    variables = {
        "PLUGIN_NAME": plugin_name,
        "PLUGIN_NAME_SNAKE": plugin_name_snake,
        "NAMESPACE": namespace,
        "AUTHOR": author,
        "DESCRIPTION": description,
        "PLUGIN_TYPE": plugin_type.capitalize(),
    }
    
    print(f"Creating {plugin_type} plugin: {plugin_name}")
    print(f"  Output directory: {output_path}")
    print(f"  Template: {template_dir}")
    print()
    
    # 复制模板
    copy_template(template_dir, output_path, variables, verbose)
    
    print()
    print("✅ Plugin created successfully!")
    print()
    print("Next steps:")
    print(f"  1. Review the generated code in: {output_path}")
    print(f"  2. Implement your algorithm in: {output_path}/algorithm/{plugin_name_snake}.cpp")
    print(f"  3. Add the plugin to CMakeLists.txt:")
    print(f"     add_subdirectory({output_path})")
    print(f"  4. Build the plugin:")
    print(f"     cd build && make {plugin_name_snake}_plugin")
    print(f"  5. Test the plugin:")
    print(f"     ./build/navsim_local_debug --planner {plugin_name} --scenario scenarios/simple_corridor.json")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="NavSim Plugin Scaffolding Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Create a planner plugin
  python3 tools/navsim_create_plugin.py \\
      --name MyPlanner \\
      --type planner \\
      --output plugins/planning/my_planner \\
      --author "Your Name" \\
      --description "My awesome planner"
  
  # Create a perception plugin
  python3 tools/navsim_create_plugin.py \\
      --name MyMapBuilder \\
      --type perception \\
      --output plugins/perception/my_map_builder
        """
    )
    
    parser.add_argument(
        "--name",
        required=True,
        help="Plugin name (CamelCase, e.g., MyPlanner)"
    )
    
    parser.add_argument(
        "--type",
        required=True,
        choices=["planner", "perception"],
        help="Plugin type"
    )
    
    parser.add_argument(
        "--output",
        required=True,
        help="Output directory (e.g., plugins/planning/my_planner)"
    )
    
    parser.add_argument(
        "--author",
        default="NavSim Developer",
        help="Author name (default: NavSim Developer)"
    )
    
    parser.add_argument(
        "--description",
        default="",
        help="Plugin description"
    )
    
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Verbose output"
    )
    
    args = parser.parse_args()
    
    try:
        create_plugin(
            plugin_name=args.name,
            plugin_type=args.type,
            output_dir=args.output,
            author=args.author,
            description=args.description,
            verbose=args.verbose
        )
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())

