#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

export DISPLAY="${DISPLAY:-:0}"
AUTH="${XAUTHORITY_HOST:-${XAUTHORITY:-$HOME/.Xauthority}}"
export XAUTHORITY_HOST="$AUTH"

if [[ ! -d /tmp/.X11-unix ]]; then
  echo "Нет /tmp/.X11-unix — нет локального X11 (SSH без -X или не Linux desktop)." >&2
  exit 1
fi

if [[ ! -f "$AUTH" ]]; then
  echo "Не найден $AUTH — без cookie X11 доступ к дисплею закроют." >&2
  exit 1
fi

# Loose local access so root in container can attach to host XWayland/Xorg (combine with compose hostname mount).
xhost +local:root 2>/dev/null || \
  xhost +SI:localuser:root 2>/dev/null || \
  xhost +local:docker 2>/dev/null || \
  xhost +local: 2>/dev/null || \
  true

# Явно передать DISPLAY — даже если контейнер подняли без него в env.
exec docker compose exec -it \
  -e "DISPLAY=$DISPLAY" \
  -e "QT_QPA_PLATFORM=xcb" \
  -e "XAUTHORITY=/tmp/.x11-docker-authority" \
  realsense \
  realsense-viewer "$@"
