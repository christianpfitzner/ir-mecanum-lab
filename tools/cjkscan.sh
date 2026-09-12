#!/usr/bin/env bash
# Finds characters from foreign scripts (CJK/Cyrillic) that text generation occasionally
# drops, single characters at a time, into the middle of an otherwise plain comment line.
# Use: tools/cjkscan.sh [path]   -> reports file:line, exit 1 on hits
cd "$(dirname "$0")/.."; target="${1:-.}"
out=$(grep -rnP "[\x{2E80}-\x{9FFF}\x{3040}-\x{30FF}\x{0400}-\x{04FF}\x{AC00}-\x{D7AF}]" \
        --include='*.py' --include='*.tex' --include='*.md' --include='*.json' \
        --include='*.sh' --include='*.txt' --include='*.launch.py' "$target" 2>/dev/null)
if [[ -n "$out" ]]; then echo "$out"; echo "^^ foreign script characters — please replace"; exit 1; fi
echo "no foreign script characters in ${target}"
