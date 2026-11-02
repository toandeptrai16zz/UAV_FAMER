#!/usr/bin/env python3
"""Exclusive, no-dependency UART reader for Ubuntu/Linux, STM32 115200 8N1.

NOTE: TIOCEXCL prevents *new* opens. Close existing screen/minicom/etc
before starting, or older readers can still consume some bytes.
"""
import argparse
import fcntl
import os
import select
import sys
import termios


def main():
    parser = argparse.ArgumentParser(description='Read and save STM32 bench log (exclusive Linux serial)')
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0', help='serial device')
    parser.add_argument('--output', default='uav_serial_clean.txt', help='text log file')
    args = parser.parse_args()
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        a = termios.tcgetattr(fd)
        a[0] = 0  # iflag: do not consume / translate data
        a[1] = 0  # oflag
        a[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        if hasattr(termios, 'CRTSCTS'):
            a[2] &= ~termios.CRTSCTS
        a[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        a[3] = 0  # lflag: no echo/canonical conversion
        a[4] = termios.B115200
        a[5] = termios.B115200
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
        print(f'EXCLUSIVE {args.port} 115200 8N1 -> {args.output}  (Ctrl+C to stop)', flush=True)
        chunk = bytearray()
        with open(args.output, 'w', buffering=1, encoding='utf-8') as log:
            while True:
                ready, _, _ = select.select([fd], [], [], 1.0)
                if not ready:
                    continue
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    continue
                chunk.extend(data)
                while b'\n' in chunk:
                    raw, _, rest = chunk.partition(b'\n')
                    chunk = bytearray(rest)
                    line = raw.replace(b'\r', b'').decode('utf-8', errors='replace')
                    print(line, flush=True)
                    log.write(line + '\n')
                # Bound memory if UART never sends a newline.
                if len(chunk) > 4096:
                    chunk = chunk[-4096:]
    finally:
        os.close(fd)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.', file=sys.stderr)
    except (OSError, ValueError) as exc:
        sys.exit(f'Serial error: {exc}')
