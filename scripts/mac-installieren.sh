#!/usr/bin/env bash
# SolidCraft mit Mesh2STEP auf dem Mac bauen und als „SolidCraft Mesh2STEP.app“ in den
# Programme-Ordner legen. Beim ersten Mal 10–30 Minuten, danach (Update) deutlich schneller.
#
#   bash mac-installieren.sh            bauen und installieren (bzw. aktualisieren)
#
# Der Code liegt danach in ~/SolidCraft-Mesh2STEP. Erneut ausführen = neueste Version holen.
set -euo pipefail

REPO="https://github.com/MitKIprogrammieren/mesh2step-convert.git"
BRANCH="mesh2step-convert"
SRC="$HOME/SolidCraft-Mesh2STEP"
APP_NAME="SolidCraft Mesh2STEP.app"

schritt() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
fehler() { printf '\n\033[1;31mFehler: %s\033[0m\n' "$*" >&2; exit 1; }

[ "$(uname -s)" = Darwin ] || fehler "Dieses Skript ist nur für macOS."

# 1. Apple-Entwicklerwerkzeuge (Compiler, git)
schritt "Entwicklerwerkzeuge prüfen"
if ! xcode-select -p >/dev/null 2>&1; then
  xcode-select --install || true
  fehler "Die Apple-Entwicklerwerkzeuge werden gerade installiert (Fenster beachten). Danach dieses Skript noch einmal starten."
fi

# 2. Rust
schritt "Rust prüfen"
if [ -f "$HOME/.cargo/env" ]; then . "$HOME/.cargo/env"; fi
if ! command -v cargo >/dev/null 2>&1; then
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
  . "$HOME/.cargo/env"
fi
rustup update stable >/dev/null 2>&1 || true
case "$(uname -m)" in
  arm64) ARCH=aarch64; TARGET=aarch64-apple-darwin ;;
  x86_64) ARCH=x86_64; TARGET=x86_64-apple-darwin ;;
  *) fehler "Unbekannter Prozessor: $(uname -m)" ;;
esac
rustup target add "$TARGET" >/dev/null

# 3. Code holen oder aktualisieren
schritt "Code holen"
if [ -d "$SRC/.git" ]; then
  git -C "$SRC" fetch --depth 1 origin "$BRANCH"
  git -C "$SRC" checkout -q -B "$BRANCH" FETCH_HEAD
else
  git clone --depth 1 -b "$BRANCH" "$REPO" "$SRC"
fi
echo "Stand: $(git -C "$SRC" log -1 --format='%h – %s')"

# 4. Bauen und als App verpacken (SolidCrafts eigenes Mac-Paketskript)
schritt "Bauen (beim ersten Mal 10–30 Minuten, der Lüfter darf laufen)"
"$SRC/packaging/macos/package.sh" --arch "$ARCH"
BUILT="$SRC/target/macos-package/SolidCraft.app"
[ -d "$BUILT" ] || fehler "Die App wurde nicht gebaut (siehe Meldungen oben)."

# 5. In den Programme-Ordner legen
schritt "Installieren nach /Applications/$APP_NAME"
DEST="/Applications/$APP_NAME"
if [ -w /Applications ]; then SUDO=""; else SUDO="sudo"; fi
$SUDO rm -rf "$DEST"
$SUDO ditto "$BUILT" "$DEST"
# Selbst gebaut, also keine Download-Sperre nötig
$SUDO xattr -dr com.apple.quarantine "$DEST" 2>/dev/null || true

schritt "Fertig"
echo "„$APP_NAME“ liegt im Programme-Ordner und lässt sich wie jede App starten."
echo "Mesh umwandeln: Körper wählen → Werkzeugleiste „Surface“ → „Convert Mesh“ → Methode „Automatic“."
open -R "$DEST" || true
