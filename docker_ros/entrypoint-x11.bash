#!/bin/bash
# Sourced by entrypoint *.bash before ros2.
#
# Compose sets XAUTHORITY=/tmp/.x11-docker-authority (writable). Host cookies mount at /root/.Xauthority.
# Typical failure: DISPLAY=:0 in compose while ~/.Xauthority only has unix:10 (XWayland etc.) —
# GUI then prints "Authorization required, but no authorization protocol specified".
#
# We pick the first DISPLAY that has a usable cookie and a matching socket under /tmp/.X11-unix, merge
# that entry into XAUTHORITY, and write /run/aida-x11.env so shells from "docker compose exec"
# inherit DISPLAY + XAUTHORITY (Compose cannot reflect PID1-time discovery).

AIDABOT_XENV="/run/aida-x11.env"
X11_AUTH_MERGED="${XAUTHORITY:-/tmp/.x11-docker-authority}"

# xauth "list :N" sometimes misses entries keyed as host/unix:N or host:0 — then we fall
# through to DISPLAY=:0 with no cookie and libX11 prints "Authorization required...".
_cookie_for_display_num() {
    local num="$1" dpy rest
    while read -r dpy rest; do
        [[ "$rest" == *MIT-MAGIC-COOKIE* ]] || continue
        if [[ "$dpy" =~ unix:${num}$ ]] || [[ "$dpy" =~ :${num}$ ]]; then
            return 0
        fi
    done < <(xauth -f /root/.Xauthority list 2>/dev/null)
    return 1
}

_display_has_cookie_and_socket() {
    local xd num="$1"
    [[ -n "$num" ]] || return 1
    [[ "${num}" == :* ]] && num="${num#:}"
    case "$num" in (*[!0-9]*) return 1 ;; esac
    xd=":${num}"
    [[ -S "/tmp/.X11-unix/X${num}" ]] || return 1
    if xauth -f /root/.Xauthority list "$xd" 2>/dev/null | grep -q MIT-MAGIC-COOKIE; then
        return 0
    fi
    _cookie_for_display_num "$num"
}

_pick_display() {
    local xd sock num

    if [[ -n "${DISPLAY:-}" ]]; then
        xd="${DISPLAY}"
        [[ "${xd}" == :* ]] || xd=":${xd}"
        if _display_has_cookie_and_socket "$xd"; then
            printf '%s' "$xd"
            return 0
        fi
    fi

    # Without nullglob, a non-matching glob is one literal path and breaks ordering.
    local -a socks=()
    shopt -q nullglob && local _ng_was_on=1 || local _ng_was_on=0
    shopt -s nullglob
    socks=(/tmp/.X11-unix/X[0-9]*)
    [[ "$_ng_was_on" == 1 ]] || shopt -u nullglob

    local sock
    while IFS= read -r sock; do
        [[ -S "$sock" ]] || continue
        num="${sock#/tmp/.X11-unix/X}"
        xd=":${num}"
        case "$num" in (*[!0-9]*) continue ;; esac
        if _display_has_cookie_and_socket "$xd"; then
            printf '%s' "$xd"
            return 0
        fi
    done < <(printf '%s\n' "${socks[@]}" | sort -u | sort -V)

    if [[ -n "${DISPLAY:-}" ]]; then
        printf '%s' "${DISPLAY}"
    else
        printf '%s' ':0'
    fi
}

if [[ ! -r /root/.Xauthority ]] || [[ ! -d /tmp/.X11-unix ]]; then
    return 0 2>/dev/null || exit 0
fi
command -v xauth >/dev/null 2>&1 || return 0 2>/dev/null || exit 0

_disp="$(_pick_display)"
export DISPLAY="$_disp"

rm -f "$X11_AUTH_MERGED" "$AIDABOT_XENV"
umask 077
touch "$X11_AUTH_MERGED"

if [[ "$_disp" != : ]] && xauth -f /root/.Xauthority nlist "$_disp" 2>/dev/null |
    xauth -f "$X11_AUTH_MERGED" nmerge - 2>/dev/null; then
    :
elif xauth -f /root/.Xauthority nlist 2>/dev/null | sed -e 's/^..../ffff/' |
    xauth -f "$X11_AUTH_MERGED" nmerge - 2>/dev/null; then
    :
else
    cp -f /root/.Xauthority "$X11_AUTH_MERGED" 2>/dev/null || touch "$X11_AUTH_MERGED"
fi

if ! xauth -f "$X11_AUTH_MERGED" list 2>/dev/null | grep -q MIT-MAGIC-COOKIE; then
    echo "aida-x11: no MIT-MAGIC-COOKIE in merged XAUTHORITY (${X11_AUTH_MERGED})." >&2
    echo "  On Wayland, in the same desktop session run: echo \"\$XAUTHORITY\"" >&2
    echo "  then either export that var before compose, or:" >&2
    echo "  XAUTHORITY_HOST=\"\$XAUTHORITY\" docker compose --profile viz up rviz" >&2
fi

umask 077
printf 'export DISPLAY=%q\nexport XAUTHORITY=%q\n' "$_disp" "$X11_AUTH_MERGED" >"$AIDABOT_XENV"
chmod 644 "$AIDABOT_XENV" 2>/dev/null || true

unset _disp
