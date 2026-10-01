#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "请使用 sudo bash deployment/systemd/install_service.sh"
    exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/../.." && pwd)"
template_path="${script_dir}/robot-runtime.service.in"
service_path="/etc/systemd/system/robot-runtime.service"
python_path="$(command -v python3 || true)"

if [[ -n "${ROBOT_SERVICE_USER:-}" ]]; then
    service_user="${ROBOT_SERVICE_USER}"
elif [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
    service_user="${SUDO_USER}"
else
    echo "无法确定普通用户。请设置 ROBOT_SERVICE_USER 后重试。"
    exit 1
fi

if [[ -z "${python_path}" ]]; then
    echo "找不到 python3，请先安装 Python 3。"
    exit 1
fi

if [[ "${project_dir}" == *"|"* || "${project_dir}" == *"&"* ]]; then
    echo "项目路径不能包含 | 或 &：${project_dir}"
    exit 1
fi

temporary_service="$(mktemp)"
trap 'rm -f "${temporary_service}"' EXIT

sed \
    -e "s|@USER@|${service_user}|g" \
    -e "s|@PROJECT_DIR@|${project_dir}|g" \
    -e "s|@PYTHON@|${python_path}|g" \
    "${template_path}" > "${temporary_service}"

install -m 0644 "${temporary_service}" "${service_path}"
systemctl daemon-reload
systemctl enable --now robot-runtime.service

echo "已安装并启动 robot-runtime.service"
echo "查看状态：systemctl status robot-runtime.service"
echo "查看日志：journalctl -u robot-runtime.service -f"
