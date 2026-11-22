#!/bin/sh
# NavPy serial-step helper gate: no two helpers, and no helper and the main
# loop, run at once, with helper jitter on and a helper descheduled between
# registering a wait and waiting. Exit 0 = no fault seen. "run.sh N old"
# runs the pre-fix wait pattern, which must fail. Then a helper's blocking
# lock take against the main thread holding that lock across clock steps
# (harness_lock.cpp), unless "old" was given.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
g++ -O1 -std=c++17 -pthread -I"$here/shim" -I"$here/../.." "$here/harness.cpp" "$here/../../NavPyStep.cpp" -o "$out/harness"
g++ -O1 -std=c++17 -pthread -I"$here/shim" -I"$here/../.." "$here/harness_lock.cpp" "$here/../../NavPyStep.cpp" -o "$out/harness_lock"
rc=0
"$out/harness" "${1:-20000}" "${2:-}" || rc=$?
if [ "$rc" = 0 ] && [ "${2:-}" != old ]; then
    "$out/harness_lock" || rc=$?
fi
rm -rf "$out"
exit $rc
