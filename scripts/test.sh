#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${URAN_PYTHON:-}"
if [[ -z "$python_bin" ]]; then
  if [[ -x /home/feiyu/miniconda3/bin/python3.13 ]]; then
    python_bin=/home/feiyu/miniconda3/bin/python3.13
  else
    python_bin="$(command -v python3)"
  fi
fi
framework_source="${URAN_FRAMEWORK_SOURCE:-$repo_dir/../uran_framework}"
interfaces_source="${URAN_INTERFACES_SOURCE:-$repo_dir/../uran_interfaces}"
wheelhouse="${URAN_DEPENDENCY_WHEELHOUSE:-}"
test_env="$(mktemp -d)"
trap 'rm -rf "$test_env"' EXIT
clean_python_artifacts() {
  "$python_bin" - "$repo_dir" <<'PY'
import shutil
import sys
from pathlib import Path
root = Path(sys.argv[1])
for path in root.rglob('__pycache__'):
    shutil.rmtree(path, ignore_errors=True)
for path in root.rglob('.pytest_cache'):
    shutil.rmtree(path, ignore_errors=True)
for path in root.rglob('*.egg-info'):
    shutil.rmtree(path, ignore_errors=True)
for path in root.rglob('build'):
    if path.is_dir() and path.parent != root:
        shutil.rmtree(path, ignore_errors=True)
PY
}
clean_python_artifacts
mkdir -p "$repo_dir/dist"
"$python_bin" - "$repo_dir/dist" <<'PY'
import sys
from pathlib import Path
for path in Path(sys.argv[1]).glob("*.whl"):
    path.unlink()
PY
"$python_bin" -m venv "$test_env"
"$test_env/bin/pip" install --disable-pip-version-check pytest PyYAML build hatchling setuptools
dependency_specs=()
dependency_options=()
if [[ -n "$wheelhouse" ]]; then
  dependency_options+=(--no-index --find-links "$wheelhouse")
  dependency_specs+=("uran-worker-sdk>=1,<2" "uran-interfaces>=1,<2")
elif [[ -n "${URAN_FRAMEWORK_WHEEL:-}" || -n "${URAN_INTERFACES_WHEEL:-}" ]]; then
  [[ -n "${URAN_FRAMEWORK_WHEEL:-}" && -n "${URAN_INTERFACES_WHEEL:-}" ]] || {
    echo "URAN_FRAMEWORK_WHEEL and URAN_INTERFACES_WHEEL must be provided together" >&2
    exit 2
  }
  dependency_specs+=("$URAN_FRAMEWORK_WHEEL" "$URAN_INTERFACES_WHEEL")
elif [[ "${URAN_ALLOW_SOURCE_DEPS:-0}" == "1" ]]; then
  dependency_specs+=("$framework_source/python/uran_worker_sdk" "$interfaces_source")
else
  dependency_specs+=("uran-worker-sdk>=1,<2" "uran-interfaces>=1,<2")
fi
"$test_env/bin/pip" install --no-build-isolation \
  "${dependency_options[@]}" "${dependency_specs[@]}"
"$test_env/bin/pip" install --no-build-isolation --no-deps \
  "$repo_dir/protocols/mqtt" "$repo_dir/motion/cyberdog2" \
  "$repo_dir/motion/control_arbiter" "$repo_dir/motion/px4" "$repo_dir/media" \
  "$repo_dir/sensors/localization" "$repo_dir/sensors/navigation" \
  "$repo_dir/tasks/autotask" "$repo_dir/simulation"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$test_env/bin/python" -m pytest --import-mode=importlib "$repo_dir"
clean_python_artifacts
for package in protocols/mqtt motion/cyberdog2 motion/control_arbiter motion/px4 media sensors/localization sensors/navigation tasks/autotask simulation; do
  clean_python_artifacts
  "$test_env/bin/python" -m build --wheel --no-isolation --outdir "$repo_dir/dist" "$repo_dir/$package"
done
clean_python_artifacts
