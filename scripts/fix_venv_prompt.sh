#!/usr/bin/env bash
set -euo pipefail

PROMPT_NAME="${1:-osl-spring-testbed}"
VENV_DIR="${2:-.venv}"

python - "$PROMPT_NAME" "$VENV_DIR" <<'PY'
from pathlib import Path
import sys

prompt = sys.argv[1]
venv = Path(sys.argv[2])

replacements = {
    venv / "bin" / "activate": [
        ('if [ "x" != x ] ; then\n    VIRTUAL_ENV_PROMPT=""\nelse\n    VIRTUAL_ENV_PROMPT=$(basename "$VIRTUAL_ENV")\nfi',
         f'VIRTUAL_ENV_PROMPT="{prompt}"'),
    ],
    venv / "bin" / "activate.fish": [
        ("if test -n ''\n    set -gx VIRTUAL_ENV_PROMPT ''\nelse\n    set -gx VIRTUAL_ENV_PROMPT (basename \"$VIRTUAL_ENV\")\nend",
         f"set -gx VIRTUAL_ENV_PROMPT '{prompt}'"),
    ],
    venv / "bin" / "activate.csh": [
        ("if ('' != \"\") then\n    setenv VIRTUAL_ENV_PROMPT ''\nelse\n    setenv VIRTUAL_ENV_PROMPT \"$VIRTUAL_ENV:t:q\"\nendif",
         f"setenv VIRTUAL_ENV_PROMPT '{prompt}'"),
    ],
    venv / "bin" / "activate.ps1": [
        ('if ("" -ne "") {\n    $env:VIRTUAL_ENV_PROMPT = ""\n}\nelse {\n    $env:VIRTUAL_ENV_PROMPT = $( Split-Path $env:VIRTUAL_ENV -Leaf )\n}',
         f'$env:VIRTUAL_ENV_PROMPT = "{prompt}"'),
    ],
}

for path, changes in replacements.items():
    if not path.exists():
        continue
    text = path.read_text()
    for old, new in changes:
        if old in text:
            text = text.replace(old, new)
    path.write_text(text)

cfg = venv / "pyvenv.cfg"
if cfg.exists():
    lines = [line for line in cfg.read_text().splitlines() if not line.startswith("prompt =")]
    lines.append(f"prompt = {prompt}")
    cfg.write_text("\n".join(lines) + "\n")
PY
