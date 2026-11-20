#!/usr/bin/env bash
# The standalone CPython carries a Clang flag unavailable in the board's GCC 7.
args=()
for arg in "$@"; do
    if [[ $arg == -fstack-clash-protection ]]; then arg=-fstack-check; fi
    args+=("$arg")
done
exec /usr/bin/g++ "${args[@]}"
