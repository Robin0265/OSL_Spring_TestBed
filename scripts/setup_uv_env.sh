#!/usr/bin/env bash
set -euo pipefail

uv venv --python /usr/bin/python --system-site-packages --prompt osl-spring-testbed --clear .venv
uv sync
scripts/fix_venv_prompt.sh
