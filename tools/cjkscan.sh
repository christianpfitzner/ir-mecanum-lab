#!/usr/bin/env bash
# Findet Zeichen aus fremden Schriftsystemen (CJK/Kyrillisch), die bei der
# Texterzeugung gelegentlich als Einzelzeichen mitten in deutsche Kommentare rutschen.
# Use: tools/cjkscan.sh [pfad]   -> meldet Datei:Zeile, Exit 1 wenn Funde
cd "$(dirname "$0")/.."; target="${1:-.}"
out=$(grep -rnP "[\x{2E80}-\x{9FFF}\x{3040}-\x{30FF}\x{0400}-\x{04FF}\x{AC00}-\x{D7AF}]" \
        --include='*.py' --include='*.tex' --include='*.md' --include='*.json' \
        --include='*.sh' --include='*.txt' --include='*.launch.py' "$target" 2>/dev/null)
if [[ -n "$out" ]]; then echo "$out"; echo "^^ fremde Schriftzeichen — bitte ersetzen"; exit 1; fi
echo "keine fremden Schriftzeichen in ${target}"
