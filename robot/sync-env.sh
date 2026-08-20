#!/usr/bin/env bash
# Add keys that exist in video.env.example but not in video.env, keeping every value you
# already set.
#
# WHY: video.env is gitignored on purpose — a `git pull` must never overwrite the address
# this robot reports to. The side effect is that new knobs added upstream only ever appear
# in the .example, so after a pull your video.env silently lacks them. They still work
# (run-video.sh has defaults) but they are invisible and untunable, which is worse than
# either extreme.
#
#   bash robot/sync-env.sh
#   sudo systemctl restart robot-video
set -euo pipefail
cd "$(dirname "$0")"

EXAMPLE="video.env.example"
TARGET="video.env"

[ -f "$EXAMPLE" ] || { echo "missing $EXAMPLE" >&2; exit 1; }

if [ ! -f "$TARGET" ]; then
  cp "$EXAMPLE" "$TARGET"
  echo "created $TARGET from the example — set PUBLISH_HOST before restarting"
  exit 0
fi

added=0
while IFS= read -r line; do
  case "$line" in
    ''|'#'*) continue ;;
  esac
  key="${line%%=*}"
  # Anchored match, so PUBLISH_HOST is not considered present because of PUBLISH_PORT.
  if ! grep -qE "^[[:space:]]*${key}=" "$TARGET"; then
    [ "$added" = 0 ] && printf '\n# --- added by sync-env.sh ---\n' >> "$TARGET"
    echo "$line" >> "$TARGET"
    echo "  + $line"
    added=$((added + 1))
  fi
done < "$EXAMPLE"

if [ "$added" = 0 ]; then
  echo "$TARGET already has every key from the example"
else
  echo "added $added key(s) with their default values — review them, then:"
  echo "  sudo systemctl restart robot-video"
fi
