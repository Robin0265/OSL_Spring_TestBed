# OSL_Spring_TestBed

## Pixi environment

For Windows development, use Pixi instead of the old `uv` environment:

```bash
pixi run python -s --version
pixi run python -s test_calibration_plotter.py
```

This workspace is pinned to Python 3.12 in `pixi.toml`, and `-s` is important on this machine because the Windows user site-packages contain a separate NumPy install that can override the Pixi environment if user-site imports are left enabled.

## Raspberry Pi camera environment

This project uses the Raspberry Pi OS `picamera2`/`libcamera` bindings, which are Linux-only and are provided by the system Python packages rather than PyPI. `picamera2` is not managed by `uv` for this project.

Recreate the uv environment on the Raspberry Pi with system site packages enabled:

```bash
uv venv --python /usr/bin/python --system-site-packages --prompt osl-spring-testbed --clear .venv
uv sync
```

The resulting `.venv/pyvenv.cfg` should include:

```ini
include-system-site-packages = true
prompt = osl-spring-testbed
```

Camera scripts should be run on the Raspberry Pi system Python environment where `picamera2` is already installed.
