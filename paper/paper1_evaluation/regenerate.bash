#!/usr/bin/env bash
# Offline figure/table regeneration only; never starts ROS or Gazebo.
set -euo pipefail
cd "$(dirname "$0")/../.."
paper_python="${PAPER1_PYTHON:-/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python}"
export PYTHONPATH="$PWD/src:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
"$paper_python" scripts/build_paper1_evaluation_package.py
"$paper_python" scripts/paper1_qualitative_figures.py --results outputs/paper1-final-eval-v1 --outdir paper/paper1_evaluation
