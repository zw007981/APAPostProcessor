#!/usr/bin/env python3
"""递归删除仓库中所有 NTFS 备用数据流标记文件（*:Zone.Identifier）。"""

import os
from pathlib import Path
from typing import List


ZONE_IDENTIFIER_SUFFIX = ":Zone.Identifier"
"""需要匹配并删除的文件名后缀。"""


def findZoneIdentifierFiles(repo_root: Path) -> List[Path]:
    """递归遍历仓库根目录，返回所有 Zone.Identifier 文件路径。"""
    return [path for path in repo_root.rglob("*") if path.is_file() and path.name.endswith(ZONE_IDENTIFIER_SUFFIX)]


def removeFiles(file_paths: List[Path]) -> int:
    """删除给定文件列表，返回成功删除的数量。"""
    removed_count = 0
    for file_path in file_paths:
        try:
            os.remove(file_path)
            print(f"已删除: {file_path}")
            removed_count += 1
        except OSError as e:
            print(f"删除失败: {file_path} ({e})")
    return removed_count


def main() -> None:
    """脚本入口：定位并清理仓库内的 Zone.Identifier 文件。"""
    repo_root = Path(__file__).resolve().parent.parent
    target_files = findZoneIdentifierFiles(repo_root)

    if not target_files:
        print("未发现 Zone.Identifier 文件。")
        return

    removed = removeFiles(target_files)
    print(f"共删除 {removed}/{len(target_files)} 个 Zone.Identifier 文件。")


if __name__ == "__main__":
    main()
