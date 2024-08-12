import os


def count_lines_of_code(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
            lines = file.readlines()
            code_lines = [line for line in lines if line.strip() and not line.strip().startswith('#')]
            return len(code_lines)
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return 0


def count_module_lines(module_path, extensions):
    total_lines = 0
    for root, _, files in os.walk(module_path):
        for file in files:
            if any(file.endswith(ext) for ext in extensions):
                file_path = os.path.join(root, file)
                total_lines += count_lines_of_code(file_path)
    return total_lines


def generate_markdown_report(target_directory, extensions, output_file):
    report_lines = []
    for module in os.listdir(target_directory):
        module_path = os.path.join(target_directory, module)
        if os.path.isdir(module_path):
            lines_count = count_module_lines(module_path, extensions)
            report_lines.append(f'- `{module}`: {lines_count}')

    with open(output_file, 'w', encoding='utf-8') as file:
        file.write('\n'.join(report_lines))


if __name__ == '__main__':
    target_directory = "./"
    extensions = ["c", "cpp", "h", "hpp", "py"]
    output_file = 'code_report.md'
    generate_markdown_report(target_directory, extensions, output_file)
    print(f"Report generated and saved to {output_file}")

