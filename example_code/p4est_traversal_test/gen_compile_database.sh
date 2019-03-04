#!/bin/bash
sed -e '1s/^/[\n/' -e '$s/,$/\n]/' obj/*.o.json > compile_commands.json
