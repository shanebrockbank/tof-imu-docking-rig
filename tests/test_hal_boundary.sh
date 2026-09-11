#!/usr/bin/env bash
# Fails if any file other than hal/hal_esp32.c includes an ESP-IDF header.
# See CLAUDE.md constraint 1 / docs/design.md §2 item 3.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PATTERN='#include\s*[<"](driver/|esp_[a-z_]*\.h|freertos/|sdkconfig\.h)'

violations=$(grep -rlE "$PATTERN" \
    --include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' \
    --exclude-dir=hardware_bringup \
    . 2>/dev/null | grep -v '^\./hal/hal_esp32\.c$' || true)

if [ -n "$violations" ]; then
    echo "HAL boundary violation: ESP-IDF headers included outside hal/hal_esp32.c:"
    echo "$violations"
    exit 1
fi

echo "HAL boundary OK: no ESP-IDF includes outside hal/hal_esp32.c"
exit 0
