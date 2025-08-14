#!/usr/bin/env bash
# 构建并安装 core 目录中的 C++ 静态库 trg_planner_core
# 使用:
#   bash shellscripts/build_core.sh                 # 默认 Release 构建并安装到 ./install (增量)
#   BUILD_TYPE=Debug bash shellscripts/build_core.sh
#   INSTALL_PREFIX=/opt/trg_planner bash shellscripts/build_core.sh
#   bash shellscripts/build_core.sh --clean         # 清理后再退出 (同时清理 build 与 install)
#   bash shellscripts/build_core.sh -c              # 同上
# 变量:
#   BUILD_TYPE      默认为 Release (可选: Debug RelWithDebInfo MinSizeRel)
#   INSTALL_PREFIX  安装前缀(默认: 项目根目录下 install/)
set -euo pipefail

usage() {
  echo "用法: $0 [--clean|-c|--help|-h]"
  echo "  --clean,-c : 删除构建(build)与安装(install)目录后退出"
  echo "  --help,-h  : 显示本帮助"
}

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CORE_DIR="${PROJECT_ROOT}/core"
BUILD_DIR="${CORE_DIR}/build"
BUILD_TYPE=${BUILD_TYPE:-Release}
INSTALL_PREFIX=${INSTALL_PREFIX:-"${PROJECT_ROOT}/install"}

if [[ ${1:-} =~ ^(--help|-h)$ ]]; then
  usage; exit 0;
fi

if [[ ${1:-} =~ ^(--clean|-c|clean)$ ]]; then
  echo "[INFO] 清理 ${BUILD_DIR} 与 ${INSTALL_PREFIX}"
  # 保护: 避免误删根目录或 /usr 等危险路径
  for dir in "${BUILD_DIR}" "${INSTALL_PREFIX}"; do
    if [[ -n "$dir" && "$dir" != "/" && "$dir" == ${PROJECT_ROOT}/* ]]; then
      rm -rf "$dir"
      echo "[INFO] 已删除 $dir"
    else
      echo "[WARN] 跳过可疑路径: $dir"
    fi
  done
  echo "[INFO] 已完成清理."
  exit 0
fi

mkdir -p "${BUILD_DIR}" "${INSTALL_PREFIX}"
cd "${BUILD_DIR}" || exit 1

echo "[INFO] 配置 (BUILD_TYPE=${BUILD_TYPE}, INSTALL_PREFIX=${INSTALL_PREFIX})"
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
      ..

echo "[INFO] 编译 trg_planner_core"
cmake --build . --config "${BUILD_TYPE}" -j"$(nproc)"

echo "[INFO] 安装到 ${INSTALL_PREFIX}"
cmake --install . --config "${BUILD_TYPE}"

echo "[SUCCESS] 完成. 头文件在 ${INSTALL_PREFIX}/include, 库在 ${INSTALL_PREFIX}/lib"
