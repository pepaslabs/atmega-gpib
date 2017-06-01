#!/bin/bash

set -e

rm -fv *.bak *.kicad_pcb-bak *-cache.lib

cd parts
./clean.sh
