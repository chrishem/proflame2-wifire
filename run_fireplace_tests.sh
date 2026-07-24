#!/usr/bin/env bash
#
# Interactive randomized test harness for test_fireplace.py.
#
# Steps through N rounds of randomized command combinations, shows the user
# what was sent, asks whether the fireplace responded normally, and logs
# each round's result to a timestamped text file for later review.
#
# Usage:
#   ./run_fireplace_tests.sh [num_rounds]
#
# Defaults to 18 rounds if not specified.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIREPLACE_SCRIPT="$SCRIPT_DIR/test_fireplace.py"
NUM_ROUNDS="${1:-18}"
RESULTS_FILE="$SCRIPT_DIR/test_results_$(date +%Y%m%d_%H%M%S).txt"

if [[ ! -f "$FIREPLACE_SCRIPT" ]]; then
    echo "Can't find $FIREPLACE_SCRIPT - run this from the same directory, or fix SCRIPT_DIR."
    exit 1
fi

echo "=== Fireplace command test harness ===" | tee "$RESULTS_FILE"
echo "Started: $(date)" | tee -a "$RESULTS_FILE"
echo "Rounds: $NUM_ROUNDS" | tee -a "$RESULTS_FILE"
echo "" | tee -a "$RESULTS_FILE"

echo "Before we start: please look at the fireplace now and note its actual"
echo "current state (this script has no way to read it back electronically)."
read -rp "Describe current state (e.g. 'off' or 'on, flame 3, fan 2'): " baseline_state
echo "Baseline state (user-reported): $baseline_state" | tee -a "$RESULTS_FILE"
echo "" | tee -a "$RESULTS_FILE"

# Track what we last *commanded*, for display purposes only - this is our
# intent, not a confirmed reading, since we have no receiver.
last_commanded="unknown (see baseline above)"

for round in $(seq 1 "$NUM_ROUNDS"); do
    echo ""
    echo "--- Round $round/$NUM_ROUNDS ---"
    echo "Last commanded state: $last_commanded"

    # Randomize: ~60% power on, ~40% power off
    if (( RANDOM % 10 < 6 )); then
        power="on"
        flame=$(( (RANDOM % 6) + 1 ))   # 1-6, 0 is blocked by test_fireplace.py anyway
        fan=$(( RANDOM % 7 ))            # 0-6
        light=$(( RANDOM % 7 ))          # 0-6
        if (( RANDOM % 2 == 0 )); then
            aux_flag="--aux"
            aux_desc="on"
        else
            aux_flag=""
            aux_desc="off"
        fi
        cmd_desc="power=on flame=$flame fan=$fan light=$light aux=$aux_desc"
        cmd_args=(--power on --flame "$flame" --fan "$fan" --light "$light")
        if [[ -n "$aux_flag" ]]; then
            cmd_args+=("$aux_flag")
        fi
    else
        power="off"
        cmd_desc="power=off"
        cmd_args=(--power off)
    fi

    echo "Sending: $FIREPLACE_SCRIPT ${cmd_args[*]} --fast"
    echo "  ($cmd_desc)"

    # test_fireplace.py re-execs itself under sudo if needed; sudo will cache
    # credentials after the first prompt so this shouldn't re-prompt every round.
    # --fast keeps batch output short (implies quiet); the RESULT: line is the
    # source of truth for what was actually sent, not our own cmd_desc reconstruction.
    set +e
    script_output=$("$FIREPLACE_SCRIPT" "${cmd_args[@]}" --fast 2>&1)
    exit_code=$?
    set -e

    echo "$script_output"
    result_line=$(echo "$script_output" | grep "^RESULT:" || echo "RESULT: (none captured)")

    if [[ $exit_code -ne 0 ]]; then
        echo "Script exited with code $exit_code (1=PWRGD failure, 2=TX failure - see output above)."
    fi

    last_commanded="$cmd_desc"

    echo ""
    read -rp "Did the fireplace respond normally? (y/n): " normal_response
    read -rp "Any notes? (optional, press enter to skip): " notes

    {
        echo "Round $round: $result_line"
        echo "  exit_code: $exit_code"
        echo "  responded_normally: $normal_response"
        echo "  notes: $notes"
        echo "  timestamp: $(date)"
        echo ""
    } >> "$RESULTS_FILE"

done

echo ""
echo "=== Done. $NUM_ROUNDS rounds completed. ==="
echo "Results saved to: $RESULTS_FILE"
