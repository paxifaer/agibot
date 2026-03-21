import os

# 要忽略的文件夹和文件类型
IGNORE_DIRS = {'.git', 'node_modules', '__pycache__', '.venv', 'dist', 'build'}
IGNORE_EXTS = {'.png', '.jpg', '.jpeg', '.gif', '.exe', '.dll', '.so', '.zip', '.tar'}

OUTPUT_FILE = 'project_dump.txt'


def should_ignore(file_path):
    _, ext = os.path.splitext(file_path)
    return ext.lower() in IGNORE_EXTS


def dump_project(root_dir):
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as out_file:
        for dirpath, dirnames, filenames in os.walk(root_dir):
            # 过滤目录
            dirnames[:] = [d for d in dirnames if d not in IGNORE_DIRS]

            for filename in filenames:
                file_path = os.path.join(dirpath, filename)

                if should_ignore(file_path):
                    continue

                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()

                    relative_path = os.path.relpath(file_path, root_dir)

                    out_file.write(f"\n\n===== FILE: {relative_path} =====\n\n")
                    out_file.write(content)

                except Exception as e:
                    print(f"跳过文件: {file_path}, 原因: {e}")


if __name__ == "__main__":
    dump_project(os.getcwd())
    print(f"已生成 {OUTPUT_FILE}")