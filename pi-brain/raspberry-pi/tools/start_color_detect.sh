#!/usr/bin/env bash

set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
required_files=(
    "${project_dir}/config/color.json"
    "${project_dir}/robot_perception/color/config.py"
    "${project_dir}/robot_perception/color/detector.py"
    "${project_dir}/tools/debug_color.py"
)

for required_file in "${required_files[@]}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "错误：缺少 ${required_file}" >&2
        echo "模块化版本需要同时复制配置和识别模块。" >&2
        exit 1
    fi
done

cd -- "${project_dir}" || exit 1
exec /usr/bin/python3 -m tools.debug_color
