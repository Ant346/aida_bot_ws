#!/usr/bin/env bash
# Calibrate D405 (cam0) and D435 (cam1) with TartanCalib.
# Frames come from capture_kalibr_frames.py.
#
#   ./scripts/run_tartancalib.sh --build
#   ./scripts/run_tartancalib.sh
#   ./scripts/run_tartancalib.sh calib_kalibr/20261009_120000
#
# --build only clones TartanCalib and builds the Noetic image.
# The image is large. Later runs only execute the calibration.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${TARTAN_SRC:-$ROOT/.tartancalib}"
IMAGE="${TARTAN_IMAGE:-tartancalib:noetic}"

ensure_image() {
  if [[ ! -d "$SRC/.git" ]]; then
    git clone --depth 1 https://github.com/castacks/tartancalib.git "$SRC"
  fi
  # This machine has 8 GB. catkin -j$(nproc) runs out of memory.
  if grep -q 'catkin build -j$(nproc)' "$SRC/Dockerfile_ros1_20_04"; then
    sed -i 's/catkin build -j$(nproc)/catkin build -j2/' "$SRC/Dockerfile_ros1_20_04"
  fi
  # snapshots.ros.org often drops the first apt update. Without its index,
  # python3-catkin-tools is invisible and the image build stops.
  if grep -q '^RUN apt-get update && DEBIAN_FRONTEND' "$SRC/Dockerfile_ros1_20_04"; then
    python3 - "$SRC/Dockerfile_ros1_20_04" <<'PY'
import pathlib, sys
path = pathlib.Path(sys.argv[1])
text = path.read_text()
old = "RUN apt-get update && DEBIAN_FRONTEND=noninteractive \\\n\tapt-get install -y \\\n"
new = """RUN set -eux; \\
\tfind /etc/apt/sources.list.d -type f -exec sed -i \\
\t\t's|http://snapshots.ros.org/noetic/final/ubuntu|http://packages.ros.org/ros/ubuntu|g' {} +; \\
\tapt-get update || true; \\
\tapt-get install -y --no-install-recommends ca-certificates curl gnupg dirmngr; \\
\tapt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654; \\
\tapt-key export F42ED6FBAB17C654 | gpg --dearmor \\
\t\t-o /usr/share/keyrings/ros1-snapshots-archive-keyring.gpg; \\
\tok=0; \\
\tfor i in 1 2 3 4 5; do \\
\t\tapt-get update && apt-cache show python3-catkin-tools >/dev/null && ok=1 && break; \\
\t\tsleep 15; \\
\tdone; \\
\ttest "$ok" = 1; \\
\tDEBIAN_FRONTEND=noninteractive apt-get install -y \\
"""
if old not in text:
    raise SystemExit("apt install line not found")
path.write_text(text.replace(old, new, 1))
PY
  fi
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Собираю $IMAGE. Первый раз это долго."
    docker build -t "$IMAGE" -f "$SRC/Dockerfile_ros1_20_04" "$SRC"
  fi
}

if [[ "${1:-}" == "--build" ]]; then
  ensure_image
  echo "Образ $IMAGE готов."
  exit 0
fi

if [[ $# -ge 1 ]]; then
  SESSION="$(cd "$1" && pwd)"
else
  calib_root="$ROOT/calib_kalibr"
  if [[ ! -d "$calib_root" ]]; then
    echo "Нет сессий в $calib_root. Сначала собери кадры." >&2
    exit 1
  fi
  mapfile -t sessions < <(find "$calib_root" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -n | awk '{print $2}')
  if [[ ${#sessions[@]} -eq 0 ]]; then
    echo "Нет сессий в $calib_root. Сначала собери кадры." >&2
    exit 1
  fi
  SESSION="${sessions[-1]}"
fi

if [[ ! -f "$SESSION/target.yaml" || ! -d "$SESSION/dataset" ]]; then
  echo "В $SESSION нет target.yaml или dataset/. Это не сессия захвата." >&2
  exit 1
fi

topics=()
models=()
counts=()
for cam in cam0 cam1; do
  n="$(find "$SESSION/dataset/$cam" -maxdepth 1 -name '*.png' 2>/dev/null | wc -l)"
  if [[ "$n" -gt 0 ]]; then
    topics+=("/$cam/image_raw")
    models+=("pinhole-radtan")
    counts+=("$cam=$n")
  fi
done

if [[ ${#topics[@]} -eq 0 ]]; then
  echo "В $SESSION/dataset нет PNG." >&2
  exit 1
fi

echo "Сессия: $SESSION"
echo "Кадры: ${counts[*]}"
echo "Модель: pinhole-radtan (plumb_bob, как у цветных D405 и D435)"

overlap=0
if [[ ${#topics[@]} -eq 2 ]]; then
  overlap="$(comm -12 \
    <(find "$SESSION/dataset/cam0" -maxdepth 1 -name '*.png' -printf '%f\n' | sort) \
    <(find "$SESSION/dataset/cam1" -maxdepth 1 -name '*.png' -printf '%f\n' | sort) \
    | wc -l | tr -d ' ')"
  echo "Общих меток времени: $overlap"
  if [[ "$overlap" -lt 15 ]]; then
    echo "Для пары лучше 20–40 кадров, где доска видна обеим камерам и занимает разные части картинки."
  fi
fi

# One joint run when the shutters share timestamps. Otherwise each camera alone.
runs=()
if [[ ${#topics[@]} -eq 2 && "$overlap" -gt 0 ]]; then
  runs+=("log|${topics[*]}|${models[*]}")
else
  for cam in cam0 cam1; do
    n="$(find "$SESSION/dataset/$cam" -maxdepth 1 -name '*.png' 2>/dev/null | wc -l | tr -d ' ')"
    if [[ "$n" -gt 0 ]]; then
      name="d435"
      [[ "$cam" == "cam0" ]] && name="d405"
      runs+=("${name}-|/${cam}/image_raw|pinhole-radtan")
    fi
  done
fi

ensure_image

run_spec="$(printf '%s\n' "${runs[@]}")"

# A 3x3 board has at most 32 corners. Kalibr accepts a view from
# max(rows, cols) + 1 tags, which is 16 corners. The large 11x8 board
# keeps the previous threshold of 24.
colmajor=0
if grep -q '^id_order: column' "$SESSION/target.yaml"; then
  colmajor=1
fi

min_corners=24
if python3 - "$SESSION/target.yaml" <<'PY'
import sys
cols = rows = 99
for line in open(sys.argv[1]):
    key, _, value = line.partition(":")
    if key.strip() == "tagCols":
        cols = int(value)
    elif key.strip() == "tagRows":
        rows = int(value)
raise SystemExit(0 if cols * rows <= 9 else 1)
PY
then
  min_corners=16
fi

docker run --rm \
  --entrypoint bash \
  -e MPLBACKEND=Agg \
  -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  -e "RUNS=$run_spec" \
  -e "MIN_CORNERS=$min_corners" \
  -e "APRILGRID_COLMAJOR=$colmajor" \
  -v "$SESSION:/data" \
  "$IMAGE" \
  -lc '
    set -euo pipefail
    mkdir -p /data/tartan
    rm -f /data/cameras.bag
    set +u
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    set -u
    rosrun kalibr kalibr_bagcreater --folder /data/dataset --output-bag /data/cameras.bag
    printf "%s\n" "$RUNS" | while IFS="|" read -r prefix topics models; do
      [ -n "$prefix" ] || continue
      # The 3x3 board has an empty center, so Kalibr cannot guess the
      # focal length from a complete grid. These are the TartanCalib
      # focals from the 11x8 session, cam0 = D405, cam1 = D435.
      focals=""
      for topic in $topics; do
        case "$topic" in
          /cam0/image_raw) printf -v focals "%s648.1701076442778\n" "$focals" ;;
          /cam1/image_raw) printf -v focals "%s891.9115996788142\n" "$focals" ;;
        esac
      done
      # shellcheck disable=SC2086
      printf "%s" "$focals" | rosrun kalibr tartan_calibrate \
        --bag /data/cameras.bag \
        --target /data/target.yaml \
        --topics $topics \
        --models $models \
        --min-init-corners-autocomplete "$MIN_CORNERS" \
        --mi-tol -1 \
        --min-views-outlier 8 \
        --dont-show-report \
        --log_dest "$prefix" \
        --save_dir /data/tartan/
    done
  '

echo "Результат в $SESSION/tartan/"
echo "Файл с 1 в имени — вторая итерация, её и берём. log0 — первая итерация Kalibr."
