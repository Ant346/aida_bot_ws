#!/usr/bin/env python3
"""Пробел из SSH-терминала (в том числе с мака) ставит аварийный стоп.

Клавиатура мака не видна в /dev/input робота. Этот процесс читает свой
терминал: пока окно SSH в фокусе, пробел сразу поднимает /tmp/odroid_estop
внутри контейнера, и драйвер снимает момент.
"""

import os
import sys
import tempfile
import termios
import tty

ESTOP_FLAG = '/tmp/odroid_estop'
ESTOP_REASON = '/tmp/odroid_estop_reason'


def _write_atomic(path, text):
    directory = os.path.dirname(path) or '.'
    fd, tmp = tempfile.mkstemp(prefix='.odroid-', dir=directory)
    try:
        os.write(fd, text.encode())
    finally:
        os.close(fd)
    os.replace(tmp, path)


def latch():
    _write_atomic(ESTOP_REASON, 'space from ssh\n')
    _write_atomic(ESTOP_FLAG, '1\n')


def main():
    if not sys.stdin.isatty():
        print(
            'Нужен терминал SSH. Запуск:\n'
            '  docker exec -it aida_bot_ws-odroid_calibrate-1 '
            'python3 /workspace/src/odroid_node/odroid_driver/estop_ssh.py',
            file=sys.stderr,
        )
        return 1

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(
        'Окно SSH в фокусе. Пробел снимает момент со всех моторов.\n'
        'Ctrl-C выходит, стоп при этом не снимается.',
        flush=True,
    )
    try:
        tty.setraw(fd)
        while True:
            char = sys.stdin.read(1)
            if char == ' ':
                latch()
                sys.stderr.write('\r\nПРОБЕЛ: стоп удерживается\r\n')
                sys.stderr.flush()
            elif char in ('\x03', '\x04'):
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print('\nВыход. Если стоп уже был, он остаётся.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
