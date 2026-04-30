#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$repo_root"

echo "[cleanup] Removing ROS 2 build/install/log artifacts..."
rm -rf build install log .ros colcon_logs

echo "[cleanup] Removing Python caches and artifacts..."
find . -type d -name "__pycache__" -prune -exec rm -rf {} +
find . -type d -name ".pytest_cache" -prune -exec rm -rf {} +
find . -type d -name ".mypy_cache" -prune -exec rm -rf {} +
find . -type d -name ".ruff_cache" -prune -exec rm -rf {} +
find . -type d -name "*.egg-info" -prune -exec rm -rf {} +
rm -rf .eggs dist python_build htmlcov .coverage .coverage.*
find . -type f \( -name "*.pyc" -o -name "*.pyo" -o -name "*\$py.class" \) -delete

echo "[cleanup] Done."
