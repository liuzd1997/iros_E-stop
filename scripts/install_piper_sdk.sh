#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_REF=841a625f5f4920e776f20b934eb13048b747e6d0
if [[ ! -d "$DEMO_ROOT/vendor/pyAgxArm/.git" ]]; then
  git clone https://github.com/agilexrobotics/pyAgxArm.git "$DEMO_ROOT/vendor/pyAgxArm"
  git -C "$DEMO_ROOT/vendor/pyAgxArm" checkout --detach "$SDK_REF"
fi
if [[ "$(git -C "$DEMO_ROOT/vendor/pyAgxArm" rev-parse HEAD)" != "$SDK_REF" ]]; then
  echo "Expected SDK revision $SDK_REF. Preserve local edits and check out that revision before installation." >&2
  exit 1
fi
/usr/bin/python3 -m pip install --target "$DEMO_ROOT/.deps/python" --no-build-isolation "$DEMO_ROOT/vendor/pyAgxArm"
