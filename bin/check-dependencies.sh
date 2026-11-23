#!/usr/bin/env bash

# Note: This is a prototype for how we could add static code analysis to the CI.

set -e

if [[ $# -eq 0 ]]; then
	set -- nucleo-wb55rg
fi

environments=()
for board in "$@"; do
	environments+=(-e "$board")
done

pio pkg outdated "${environments[@]}"
