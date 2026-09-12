#!/usr/bin/env bash
# Fails if any file other than hal/hal_esp32.c includes an ESP-IDF header.
# See CLAUDE.md constraint 1 / docs/design.md §2 item 3.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PATTERN='#include\s*[<"](driver/|esp_[a-z_]*\.h|freertos/|sdkconfig\.h)'

# hardware_bringup/ is standalone pre-V1 bring-up firmware, outside the V1/V2
# pipeline — see hardware_bringup/README.md for the pre-authorized exception.
# vl53l1x_uld (vendored under both hardware_bringup/ and firmware/components/)
# is a proven third-party ESP-IDF driver, not hand-written pipeline code — its
# ESP-IDF includes are expected and accepted duplication, see
# docs/superpowers/specs/2026-09-11-debt1-esp32-hal-backend-design.md §8.
violations=$(grep -rlE "$PATTERN" \
    --include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' \
    --exclude-dir=hardware_bringup \
    --exclude-dir=vl53l1x_uld \
    . 2>/dev/null | grep -v '^\./hal/hal_esp32\.c$' || true)

if [ -n "$violations" ]; then
    echo "HAL boundary violation: ESP-IDF headers included outside hal/hal_esp32.c:"
    echo "$violations"
    exit 1
fi

echo "HAL boundary OK: no ESP-IDF includes outside hal/hal_esp32.c"
exit 0
