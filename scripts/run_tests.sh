#!/usr/bin/env bash
#
# Convenience wrapper to run the project's unit & emulation tests.
#
# Usage:
#   ./scripts/run_tests.sh                # Build-only on qemu_x86 (fast, recommended on macOS)
#   ./scripts/run_tests.sh --run          # Also execute the tests (requires QEMU + x86 toolchain)
#   ./scripts/run_tests.sh --logic        # Only pure logic tests (fastest feedback)
#   ./scripts/run_tests.sh --help
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Default venv location (adjust if yours is different)
VENV_ACTIVATE="${PROJECT_ROOT}/../../.venv/bin/activate"
if [[ ! -f "$VENV_ACTIVATE" ]]; then
    VENV_ACTIVATE="/Users/rccalam/Workspace/Projects/zephyr-rtos/.venv/bin/activate"
fi

MODE="build-only"
FILTER=""
VERBOSE=""

print_help() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  --run          Build + execute tests (needs working QEMU for qemu_x86)"
    echo "  --logic        Only run the fast pure-logic tests (pump SM, flow processor, fixed math, zbus)"
    echo "  --build-only   Only compile the tests (default, fast on macOS)"
    echo "  -v, --verbose  Pass -v to twister for more output"
    echo "  -h, --help     Show this help"
    echo
    echo "Examples:"
    echo "  $0                    # Quick build check of everything that can run on this machine"
    echo "  $0 --logic            # Fastest feedback loop while developing pure logic"
    echo "  $0 --run              # Full build + execution (Linux or when x86 toolchain is ready)"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run)
            MODE="run"
            shift
            ;;
        --logic)
            FILTER="--tag logic"
            shift
            ;;
        --build-only)
            MODE="build-only"
            shift
            ;;
        -v|--verbose)
            VERBOSE="-v"
            shift
            ;;
        -h|--help)
            print_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
    esac
done

if [[ ! -f "$VENV_ACTIVATE" ]]; then
    echo "ERROR: Could not find venv activate script at:"
    echo "  $VENV_ACTIVATE"
    echo "Please adjust the path in this script or activate the venv manually."
    exit 1
fi

echo "==> Activating Zephyr venv..."
source "$VENV_ACTIVATE"

cd "$PROJECT_ROOT"

echo "==> Running tests with west twister (platform: qemu_x86, mode: ${MODE})"
echo

if [[ "$MODE" == "run" ]]; then
    west twister -T tests -p qemu_x86 --inline-logs $FILTER $VERBOSE
else
    west twister -T tests -p qemu_x86 --inline-logs --build-only $FILTER $VERBOSE
fi

echo
echo "==> Done. See twister-out/ for detailed reports (twister.json, twister.xml, etc.)"
echo "    Tip: Use 'west twister -T tests -p qemu_x86 -s <TEST_ID>' to re-run a single test."