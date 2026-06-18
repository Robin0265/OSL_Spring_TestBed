# OSL_Spring_TestBed

## Raspberry Pi camera environment

This project uses the Raspberry Pi OS `picamera2`/`libcamera` bindings, which are provided by the system Python packages rather than PyPI. Recreate the uv environment on the Raspberry Pi with system site packages enabled:

```bash
uv venv --python /usr/bin/python --system-site-packages --prompt osl-spring-testbed --clear .venv
uv sync
```

The resulting `.venv/pyvenv.cfg` should include:

```ini
include-system-site-packages = true
prompt = osl-spring-testbed
```
