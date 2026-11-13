#!/usr/bin/env bash
# Prints the table of start-pose shortcuts: for each family of autonomous route, where to set the robot up (rule G304) to save the most time,
# what the placement margin costs and how many seconds it saves against the standard start (tile B1 or C1, front on the audience wall).
#
# It compiles ONLY the pure-Java sources (BioBuzzConstants and the autoshoot core, routes and render packages) with plain javac into a temporary folder,
# runs StartShortcuts and deletes the folder again. No Gradle, no Android SDK and no robot are needed; any JDK 8 or newer will do. It takes a few minutes:
# every family of routes is tried from about 37 thousand legal starts (a place every 0.25 in along the three own walls, every 5 degrees).
#
# Usage:  tools/start-shortcuts.sh [family id]      (one of shoot_park, shoot_garden, shoot_flower, partner_park, partner_flower, leave_park; default: all)
set -eu

# always work from the repository root, wherever the script is started from
cd "$(dirname "$0")/.."

SRC="TeamCode/src/main/java"
MAIN_CLASS="org.firstinspires.ftc.teamcode.autoshoot.routes.StartShortcuts"
MAIN_FILE="$SRC/org/firstinspires/ftc/teamcode/autoshoot/routes/StartShortcuts.java"

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

BUILD="$(mktemp -d "${TMPDIR:-/tmp}/start-shortcuts.XXXXXX")"
trap 'rm -rf "$BUILD"' EXIT

"$JAVAC" -encoding UTF-8 -nowarn -sourcepath "$SRC" -d "$BUILD" "$MAIN_FILE"
"$JAVA" -cp "$BUILD" "$MAIN_CLASS" "$@"
