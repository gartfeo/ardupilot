#!/bin/sh
# NavPy serial-step helper gate: no two helpers, and no helper and the main
# loop, run at once, with helper jitter on. Exit 0 = no fault seen.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
g++ -O1 -std=c++17 -pthread -I"$here/shim" -I"$here/../.." "$here/harness.cpp" "$here/../../NavPyStep.cpp" -o "$out/harness"
"$out/harness" "${1:-20000}"
rm -rf "$out"
