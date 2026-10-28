#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"

run_limited() {
    limit=$1
    shift
    perl -e 'my $seconds = shift; alarm $seconds; exec @ARGV' "$limit" "$@"
}

run_limited 120 cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
run_limited 180 cmake --build build -j4
run_limited 30 ./build/rwb_tests
run_limited 30 ./build/rwb_shell_spec --out outputs/shell_spec.txt

run_limited 240 ./build/rwb_train \
    --samples 24000 \
    --policy-samples 8000 \
    --seed 17 \
    --output models/stage1.model

run_limited 60 ./build/rwb_benchmark \
    --model models/stage1.model \
    --episodes 40 \
    --seed 73021 \
    --out outputs/stress_round1 \
    --stress

run_limited 60 ./build/rwb_curriculum \
    --telemetry outputs/stress_round1/telemetry \
    --out outputs/stress_round1/curriculum.csv

run_limited 240 ./build/rwb_train \
    --samples 24000 \
    --policy-samples 8000 \
    --seed 53 \
    --stress \
    --mixed \
    --curriculum outputs/stress_round1/curriculum.csv \
    --output models/recoverability.model

run_limited 60 ./build/rwb_benchmark \
    --model models/recoverability.model \
    --episodes 40 \
    --seed 44017 \
    --out outputs/standard_mixed_final

run_limited 60 ./build/rwb_benchmark \
    --model models/recoverability.model \
    --episodes 40 \
    --seed 99001 \
    --out outputs/stress_mixed_final \
    --stress

printf '%s\n' "standard: outputs/standard_mixed_final"
printf '%s\n' "stress:   outputs/stress_mixed_final"
printf '%s\n' "model:    models/recoverability.model"

