#!/usr/bin/env bash
# Regenerates the autonomous route diagrams in docs/autos: index.html plus one SVG per route and alliance.
#
# It compiles ONLY the pure-Java sources (BioBuzzConstants and the autoshoot core, routes and render
# packages) with plain javac into a temporary folder, runs RenderRoutesMain and deletes the folder again.
# javac follows the imports from the entry point, so nothing else is read. No Gradle, no Android SDK
# and no robot are needed; any JDK 8 or newer will do.
#
# Usage:  tools/render-autos.sh [output directory]      (default: docs/autos)
set -eu

# always work from the repository root, wherever the script is started from
cd "$(dirname "$0")/.."

SRC="TeamCode/src/main/java"
MAIN_CLASS="org.firstinspires.ftc.teamcode.autoshoot.render.RenderRoutesMain"
MAIN_FILE="$SRC/org/firstinspires/ftc/teamcode/autoshoot/render/RenderRoutesMain.java"
OUT_DIR="${1:-docs/autos}"

# javac: JAVA_HOME if it has one, otherwise whatever is on the PATH
if [ -n "${JAVA_HOME:-}" ] && [ -x "$JAVA_HOME/bin/javac" ]; then
    JAVAC="$JAVA_HOME/bin/javac"
    JAVA="$JAVA_HOME/bin/java"
elif command -v javac >/dev/null 2>&1; then
    JAVAC="javac"
    JAVA="java"
else
    echo "javac was not found. Install a JDK (8 or newer) or set JAVA_HOME, then run this script again." >&2
    exit 1
fi

BUILD="$(mktemp -d "${TMPDIR:-/tmp}/render-autos.XXXXXX")"
trap 'rm -rf "$BUILD"' EXIT

"$JAVAC" -encoding UTF-8 -nowarn -sourcepath "$SRC" -d "$BUILD" "$MAIN_FILE"
"$JAVA" -cp "$BUILD" "$MAIN_CLASS" "$OUT_DIR"
