# Full measurement campaign: reactive against predictive, N repetitions
# each, without graphics, with the analysis at the end.
#
#   ./scripts/run_experiments.sh
#
# REPS, DURATION and TRAJ can be overridden through environment variables,
# but the defaults are the ones that produced the result reported in the
# documentation.
#
# Each run ends by itself after DURATION seconds. The repetitions matter
# because a single run says nothing: the simulation has sensor noise, and the
# exact moment the target disappears differs from one run to the next.

set -u
# Four repetitions per strategy: with three, a single bad run dominates the
# mean. With four the median survives one outlier, and it is the minimum for
# reporting a dispersion that means anything.
REPS=${REPS:-4}
# 120 s of simulated time per run: the campaign fits in about half an hour of
# real time and still collects around twenty losses per strategy, enough for a
# comparison. Longer runs add waiting, not information.
DURATION=${DURATION:-120}
TRAJ=${TRAJ:-}

# With TRAJ empty the argument must NOT be passed at all: 'trajectory:=' with
# no value is rejected by ros2 launch and the run dies before starting. Empty
# means "use the default route", which the launch file resolves.
TRAJ_ARG=""
[ -n "$TRAJ" ] && TRAJ_ARG="trajectory:=$TRAJ"
OUT=${OUT:-/root/ros_workspace/results}

mkdir -p "$OUT"
TOT=$((REPS * 2))
N=0
START=$(date +%s)

echo "campaign: $REPS repetitions x 2 strategies | route '${TRAJ:-default}', ${DURATION}s each"
echo "estimate: ~$(( TOT * DURATION * 10 / 7 / 60 )) minutes | results in $OUT"
echo

# INTERLEAVED ORDER, not all the reactive runs and then all the predictive.
#
# Gazebo leaves behind processes that do not die at once, and the next run
# starts with its transform buffer polluted by simulated timestamps from the
# previous one. The pollution GROWS along the campaign: measured on the logs,
# the first run had zero TF_OLD_DATA lines and the last nearly 35000.
#
# Run in blocks, whichever strategy goes second inherits all the degradation
# and looks worse for a reason that has nothing to do with it. Interleaving
# spreads any residue evenly.
for i in $(seq 1 "$REPS"); do
  for strategy in reactive predictive; do
    N=$((N + 1))
    echo "[$N/$TOT] repetition $i, $strategy"
    ros2 launch leader_follower main.launch.py \
        strategy:="$strategy" \
        $TRAJ_ARG \
        run_label:="${strategy}_${TRAJ:-default}" \
        output_dir:="$OUT" \
        duration:="$DURATION" \
        gui:=false rviz:=false \
        > "$OUT/${strategy}_${TRAJ:-default}_${i}.log" 2>&1

    # Force-kill whatever is left: without this the residue accumulates from
    # run to run and the last ones stop being comparable with the first.
    pkill -f "gz sim" 2>/dev/null
    pkill -f "ruby.*gz" 2>/dev/null
    pkill -f "lib/nav2_" 2>/dev/null
    for _ in $(seq 1 40); do
      pgrep -f "gz sim" >/dev/null 2>&1 || break
      sleep 0.5
    done
    # Generous wait before the next run. Two seconds were not enough: Gazebo
    # was still releasing resources and spawning the robots failed with
    # 'create has died exit code 255', losing the run.
    sleep 6
  done
done

echo
echo "campaign completed in $((($(date +%s) - START) / 60)) minutes"
echo
python3 "$(dirname "$0")/analyze_results.py" "$OUT"
