import os
import re

import pydot


def generate_dependency_graph(src_dirs, output_file='module_dependencies.png', file_types=('.cpp', '.c', '.h', '.hpp')):
    # 获取所有子目录（模块）
    modules = {}
    for src_type, src_dir in src_dirs.items():
        modules[src_type] = [d for d in os.listdir(src_dir) if os.path.isdir(os.path.join(src_dir, d))]

    # 初始化依赖关系字典
    dependencies = {f"{src_type}/{module}": set() for src_type in modules for module in modules[src_type]}

    # 定义正则表达式来匹配头文件包含语句
    include_pattern = re.compile(r'#include\s*["<](\S+)[">]')

    # 遍历每个模块，分析头文件引用
    for src_type, src_dir in src_dirs.items():
        for module in modules[src_type]:
            module_path = os.path.join(src_dir, module)
            print(f"Checking {src_type}: {module}")
            for root, _, files in os.walk(module_path):
                for file in files:
                    if file.endswith(file_types):
                        file_path = os.path.join(root, file)
                        print(f"Processing file: {file_path}")
                        with open(file_path, 'r', errors='ignore') as f:
                            content = f.read()
                            includes = include_pattern.findall(content)
                            for include in includes:
                                print(f"Found include: {include} in file: {file_path}")
                                # 检查包含的头文件是否属于其他模块或src目录的其他部分
                                for other_src_type, other_modules in modules.items():
                                    for other_module in other_modules:
                                        other_module_path = os.path.join(src_dirs[other_src_type], other_module)
                                        if include.startswith(other_module):
                                            print(f"Module {src_type}/{module} depends on {other_src_type}/{other_module}")
                                            dependencies[f"{src_type}/{module}"].add(f"{other_src_type}/{other_module}")

    # 检查是否有依赖关系
    if not any(dependencies.values()):
        print("No dependencies found.")
        return

    # 创建Graphviz有向图
    graph = pydot.Dot(graph_type='digraph', rankdir='LR', splines='true', overlap='false', sep='+1')

    # 添加节点和边
    for module, deps in dependencies.items():
        node_label = module.replace('modules/', 'module/').replace('drivers/', 'driver/')
        # 设置节点颜色
        if module.startswith('modules/'):
            node = pydot.Node(module, label=node_label, shape='box', style='filled', fillcolor='lightblue')
        else:
            node = pydot.Node(module, label=node_label, shape='box', style='filled', fillcolor='lightgrey')
        graph.add_node(node)
        for dep in deps:
            edge = pydot.Edge(module, dep)
            graph.add_edge(edge)

    # 保存依赖关系图到文件
    graph.write_png(output_file)
    print(f'依赖关系图已生成并保存为 {output_file}')


if __name__ == '__main__':
    base_url = "/home/north/PX4-Autopilot/"
    # 定义源码目录
    src_dirs = {
        'modules': os.path.join(base_url, 'src/modules'),
        'drivers': os.path.join(base_url, 'src/drivers'),
        'platforms-common': os.path.join(base_url, 'platforms/common'),
        'platforms-nuttx': os.path.join(base_url, 'platforms/nuttx'),
        'platforms-posix': os.path.join(base_url, 'platforms/posix'),
        'platforms-qurt': os.path.join(base_url, 'platforms/qurt'),
        'platforms-ros2': os.path.join(base_url, 'platforms/ros2'),
    }
    generate_dependency_graph(src_dirs, file_types=('.cpp', '.c', '.h', '.hpp'))
