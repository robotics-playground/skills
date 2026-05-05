# Final Link Verification

Before declaring the design tree done, verify every relative markdown link resolves.

## The check

```bash
cd <project-root>

fail=0
total=0
while IFS= read -r line; do
  src="${line%%:*}"
  rest="${line#*:}"
  target="${rest#*\(}"
  target="${target%%)*}"
  target="${target%% *}"
  target="${target%%#*}"
  case "$target" in
    http*|mailto*|"package://"*|"") continue ;;
    *.md|*.puml|*.svg|*.yaml|*.yml|*.txt|*/) ;;  # only markdown-relevant extensions
    *) continue ;;
  esac
  total=$((total+1))
  src_dir="$(dirname "$src")"
  resolved="$src_dir/$target"
  if [ ! -e "$resolved" ]; then
    echo "BROKEN  $src  ->  $target  (resolved: $resolved)"
    fail=$((fail+1))
  fi
done < <(grep -rn -E '\]\([^)]+\)' --include='*.md' designs/)

echo "---"
echo "checked: $total · broken: $fail"
```

## Handling false positives

The regex matches any `](something)` pattern, including parenthetical text like `(per architecture revision)`. Common false positives:

- `]("package://*"` — package URI in a parameter table. Skip via the case statement above.
- `](D1–D23)` — text reference in prose, not a link. The extension whitelist filters these.
- `](/safety/override)` — ROS topic name in prose. The extension whitelist filters these.

If a "broken" hit ends in something that's not a markdown-relevant extension, it's almost certainly a false positive — the regex over-matches. The script above filters these out.

## What to do if a link is broken

1. **Stale path** — the target file exists but at a different location. Fix the link.
2. **File not yet authored** — happens during incremental authoring. Add the file or remove the link.
3. **Typo** — fix.

Never silence a broken link by removing the surrounding sentence — that just hides design intent. Either fix the link or admit the missing file is a gap and write it.

## Verify SVG embeds in index.md

```bash
cd <designs>/diagrams

for img in $(grep -oE '<img src="[^"]+"' index.md | sed 's/<img src="//; s/"//'); do
  if [ -f "$img" ]; then echo "OK   $img"; else echo "BROKEN $img"; fi
done
```

If `index.md` references `node-map.svg` but `plantuml -tsvg` was never run, the embed will be broken. Run plantuml or note in the user-facing summary that SVG generation requires the user to install plantuml.

## Final sanity checks before commit

- [ ] Every `.puml` file has a sibling `.svg` (or you've told the user how to render them).
- [ ] Every `<img src>` in `diagrams/index.md` resolves.
- [ ] Every relative markdown link in `designs/` resolves.
- [ ] No `title` line in any `.puml` file: `grep -n "^title " designs/diagrams/*.puml` returns empty.
- [ ] No inline-alias shorthand in any `.puml`: `grep -n '"\s*as\s\+[a-z]' designs/diagrams/*.puml` returns empty.
- [ ] FR-to-package traceability in `package-structure.md` mentions every FR from the PRD.

Only after all of these pass should you tell the user the design tree is done.
