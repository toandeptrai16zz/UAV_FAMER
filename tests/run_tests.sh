#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
test_bin=$(mktemp)
trap 'rm -f "$test_bin"' EXIT
cc -std=c11 -O1 -g -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter \
  -fsanitize=undefined -fno-sanitize-recover=all -Itests/mocks -ICore/Inc \
  tests/test_flight.c Core/Src/flight_pid.c Core/Src/rc_input.c -lm -o "$test_bin"
"$test_bin"
python3 tests/test_protocol.py
