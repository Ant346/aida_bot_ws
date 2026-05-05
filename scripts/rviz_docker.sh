#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Use: ssh -Y user@host" >&2
  exit 1
fi

AUTH="${XAUTHORITY_HOST:-${XAUTHORITY:-$HOME/.Xauthority}}"
if [[ ! -f "$AUTH" ]]; then
  echo "Xauthority not found: $AUTH" >&2
  exit 1
fi
export XAUTHORITY_HOST="$AUTH"

if [[ ! -f install/setup.bash ]]; then
  echo "Missing install/setup.bash — run: docker compose up odroid_node (or realsense) once." >&2
  exit 1
fi

exec docker compose --profile viz run --rm rviz
