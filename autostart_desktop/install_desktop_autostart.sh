#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"

mkdir -p "$AUTOSTART_DIR"
tmp_nav_desktop="$(mktemp)"
tmp_record_desktop="$(mktemp)"
trap 'rm -f "$tmp_nav_desktop" "$tmp_record_desktop"' EXIT
sed "s|__WORKSPACE__|$WORKSPACE|g" "$SCRIPT_DIR/nav.desktop" >"$tmp_nav_desktop"
sed "s|__WORKSPACE__|$WORKSPACE|g" "$SCRIPT_DIR/mid360-record.desktop" >"$tmp_record_desktop"
install -m 0644 "$tmp_nav_desktop" "$AUTOSTART_DIR/pfa-navigation.desktop"
install -m 0644 "$tmp_record_desktop" "$AUTOSTART_DIR/mid360-mapping-record.desktop"

# The old entries are retained for traceability but made inert so they cannot
# start the legacy workspace in parallel with the migrated services.
for legacy in "$AUTOSTART_DIR/nav.sh.desktop" "$AUTOSTART_DIR/cd-1.desktop"; do
  if [[ -f "$legacy" ]]; then
    sed -i \
      -e 's/^X-GNOME-Autostart-enabled=.*/X-GNOME-Autostart-enabled=false/' \
      -e 's/^Hidden=.*/Hidden=true/' \
      "$legacy"
  fi
done

echo "[desktop_autostart] installed in: $AUTOSTART_DIR"
echo "[desktop_autostart] navigation: $WORKSPACE/tools/match_start/reality/nav.sh"
echo "[desktop_autostart] recorder: $WORKSPACE/autostart_mid360_record/mid360_mapping_record.sh"
