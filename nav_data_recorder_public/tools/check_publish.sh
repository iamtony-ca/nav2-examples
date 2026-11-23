#!/usr/bin/env bash
# Check this repository before pushing it to a public remote.
#   tools/check_publish.sh            (run from anywhere inside the repository)
# Fails when
#   - a profile other than profiles/default/ would be published (tracked or not ignored),
#   - a file that would be published contains a term listed in profiles/<any>/publish_terms.txt
#     (one extended regex per line, '#' comments; keep that file in the private profile),
#   - a file contains an absolute home path (/home/<user>) or an IPv4 address other than
#     127.0.0.1 / 0.0.0.0.
set -uo pipefail
top="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$top"
fail=0

if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && [ "$(git rev-parse --show-toplevel)" = "$top" ]; then
  mapfile -t files < <(git ls-files --cached --others --exclude-standard)
else
  echo "note: not the top of a git repository; checking all files except non-default profiles"
  mapfile -t files < <(find . -type f -not -path './profiles/*/*' -o -type f -path './profiles/default/*' | sed 's|^\./||' | grep -v '^\.git/')
fi

for f in "${files[@]}"; do
  if [[ "$f" == profiles/*/* && "$f" != profiles/default/* ]]; then
    echo "FAIL profile would be published: $f"; fail=1
  fi
done

terms=()
for t in profiles/*/publish_terms.txt; do
  [ -f "$t" ] || continue
  while IFS= read -r line; do
    line="${line%%#*}"; line="$(echo "$line" | xargs)"
    [ -n "$line" ] && terms+=("$line")
  done < "$t"
done

check() {  # label regex
  local hits
  hits=$(printf '%s\0' "${files[@]}" | xargs -0 grep -IinE -- "$2" 2>/dev/null | grep -vE "$3" | head -20)
  if [ -n "$hits" ]; then echo "FAIL $1:"; echo "$hits" | sed 's/^/  /' | cut -c1-200; fail=1; fi
}
skip='^tools/check_publish\.sh:'
for t in "${terms[@]}"; do check "term '$t'" "$t" "$skip"; done
check "home path" '/home/[A-Za-z0-9_]' "$skip"
check "IPv4 address" '\b([0-9]{1,3}\.){3}[0-9]{1,3}\b' "$skip|127\.0\.0\.1|0\.0\.0\.0"

echo "checked ${#files[@]} files, ${#terms[@]} private terms"
if [ "$fail" = 0 ]; then echo "OK"; else echo "NOT OK: fix the above before pushing"; fi
exit "$fail"
