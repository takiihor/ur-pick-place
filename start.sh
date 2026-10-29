#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
usage() {
  cat <<'HELP'
Usage: ./start.sh [ui|marker-teach|pick-place|vision|gripper|test|setup] [options]

  ./start.sh                              Install dependencies and open RTDE UI
  ./start.sh --demo                       Open simulated UI; no robot required
  ./start.sh --config pick_place.json     Initialize UI limits from a recipe
  ./start.sh --no-browser --port 8765      Start backend without opening browser
  ./start.sh pick-place --config pick_place.json --dry-run
  ./start.sh vision observe --config pick_place.json --vision vision.json
  ./start.sh vision vision-pick --config pick_place.json --vision vision.json
  ./start_marker_teach.sh                 Open step-by-step marker/pick teaching
  ./start.sh gripper ports
  ./start.sh test                         Run offline tests
  ./start.sh setup                        Install hardware dependencies only

Defaults to ui. UI and pick-place options are forwarded to the Python command.
Set PYTHON_BIN=/path/to/python3.12 to select an interpreter. Python 3.12 is
recommended for prebuilt ur_rtde wheels; no system packages are changed.
HELP
}
case "${1:-}" in -h|--help) usage; exit 0 ;; esac
mode=ui
case "${1:-}" in ui|marker-teach|pick-place|vision|gripper|test|setup) mode="$1"; shift ;; esac
extras=hardware
if [[ "$mode" == vision || "$mode" == marker-teach ]]; then extras=hardware,vision; fi
for arg in "$@"; do
  if [[ "$arg" == --demo && "$mode" == ui ]]; then extras=; fi
done
if [[ "$mode" == test ]]; then extras=; fi
venv_python="$project_dir/.venv/bin/python"
if [[ ! -x "$venv_python" ]]; then
  selected_python=
  if [[ -n "${PYTHON_BIN:-}" ]]; then
    selected_python="$PYTHON_BIN"
  else
    for candidate in "$HOME/.local/opt/ur-handeye-python/bin/python3.12" python3.12 python3.11 python3.10 python3.13 "$HOME"/.local/share/uv/python/cpython-3.12-*/bin/python3.12 python3; do
      if command -v "$candidate" >/dev/null 2>&1 && "$candidate" -c 'import sys; raise SystemExit(not ((3,10) <= sys.version_info[:2] <= (3,13)))' 2>/dev/null; then
        selected_python="$candidate"; break
      fi
    done
  fi
  if [[ -z "$selected_python" ]]; then
    echo 'Python 3.10–3.13 was not found. Install Python 3.12 or set PYTHON_BIN.' >&2
    exit 1
  fi
  echo "Creating .venv using $selected_python"
  "$selected_python" -m venv "$project_dir/.venv"
fi
if ! "$venv_python" -c 'import sys; raise SystemExit(sys.version_info < (3,10))'; then
  echo 'The existing .venv needs Python 3.10 or newer. Move it aside and rerun.' >&2
  exit 1
fi
if [[ "$mode" == vision || "$mode" == marker-teach ]] && ! "$venv_python" -c 'import sys; raise SystemExit(sys.version_info < (3,11))'; then
  echo 'Vision needs Python 3.11 or newer. Move .venv aside and rerun with PYTHON_BIN=python3.12.' >&2
  exit 1
fi
fingerprint="$("$venv_python" - "$extras" <<'PY'
import hashlib, pathlib, sys
print(hashlib.sha256(pathlib.Path('pyproject.toml').read_bytes() + sys.argv[1].encode() + sys.version.encode()).hexdigest())
PY
)"
stamp="$project_dir/.venv/.ur-pick-place-install-${extras:-base}"
if [[ ! -f "$stamp" || "$(cat "$stamp")" != "$fingerprint" ]] || ! "$venv_python" -c 'import ur_pick_place' >/dev/null 2>&1; then
  install_target="$project_dir"
  if [[ -n "$extras" ]]; then install_target="${project_dir}[${extras}]"; fi
  echo 'Installing standalone package dependencies…'
  "$venv_python" -m pip install --disable-pip-version-check -e "$install_target"
  printf '%s\n' "$fingerprint" > "$stamp"
fi
export PYTHONUNBUFFERED=1
case "$mode" in
  ui) exec "$venv_python" -m ur_pick_place.ui "$@" ;;
  pick-place) exec "$venv_python" -m ur_pick_place "$@" ;;
  vision|marker-teach)
    for sibling in charuco-generator charuco-realsense; do
      module="${sibling//-/_}"
      if ! "$venv_python" -c "import $module" >/dev/null 2>&1 && [[ -d "$project_dir/../$sibling" ]]; then
        echo "Installing ../$sibling (editable, --no-deps)…"
        "$venv_python" -m pip install --disable-pip-version-check --no-deps -e "$project_dir/../$sibling"
      fi
    done
    if [[ "$mode" == marker-teach ]]; then
      exec "$venv_python" -m ur_pick_place.vision.teach_ui "$@"
    fi
    exec "$venv_python" -m ur_pick_place.vision "$@" ;;
  gripper) exec "$venv_python" -m ur_pick_place.grippers.arduino_driver "$@" ;;
  test) exec "$venv_python" -m unittest discover -s tests "$@" ;;
  setup) echo 'Dependencies installed. Run ./start.sh to open Robot Bench.' ;;
esac
