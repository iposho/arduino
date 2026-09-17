#!/usr/bin/env bash
# Требует bash (массивы, [[ ]], here-string). Не запускайте через sh/dash.
if [ -z "${BASH_VERSION:-}" ]; then
  exec /usr/bin/env bash "$0" "$@"
fi

# Сборка OTA-бинарников (.ino.bin) для всех ESP32-проектов.
# Использование:
#   ./scripts/build-ota.sh           — все проекты
#   ./scripts/build-ota.sh flat      — только esp32_flat_bme280
#   ./scripts/build-ota.sh flamingo  — только esp32_flamingo
#   ./scripts/build-ota.sh default   — только esp32_default
#   ./scripts/build-ota.sh balcony cam

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OTA_DIR="$ROOT/ota"

find_arduino_cli() {
  if [[ -n "${ARDUINO_CLI:-}" && -x "$ARDUINO_CLI" ]]; then
    echo "$ARDUINO_CLI"
    return
  fi
  if command -v arduino-cli >/dev/null 2>&1; then
    command -v arduino-cli
    return
  fi
  local bundled="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
  if [[ -x "$bundled" ]]; then
    echo "$bundled"
    return
  fi
  echo "arduino-cli не найден. Установите Arduino CLI или задайте ARDUINO_CLI." >&2
  exit 1
}

ARDUINO_CLI="$(find_arduino_cli)"

read_fw_version() {
  local sketch_dir="$1"
  local info_file="$ROOT/$sketch_dir/firmware_info.h"

  if [[ ! -f "$info_file" ]]; then
    echo "unknown"
    return
  fi

  sed -n 's/^#define FW_VERSION[[:space:]]*"\([^"]*\)".*/\1/p' "$info_file" | head -1
}

ota_filename() {
  local base_name="$1"
  local sketch_dir="$2"
  local version build_date

  version="$(read_fw_version "$sketch_dir")"
  [[ -z "$version" ]] && version="unknown"
  build_date="$(date +%Y%m%d)"

  local stem="${base_name%.bin}"
  echo "${stem}-${version}-${build_date}.bin"
}

# id|sketch_dir|fqbn|build_subdir|ota_basename|partition
PROJECTS=(
  "flat|esp32_flat_bme280|esp32:esp32:esp32:PartitionScheme=default|esp32.esp32.esp32|esp32-flat.bin|default"
  "balcony|esp32_balcony_pms5003_bme280|esp32:esp32:esp32:PartitionScheme=default|esp32.esp32.esp32|esp32-balcony.bin|default"
  "cam|esp32_cam|esp32:esp32:esp32cam:PartitionScheme=min_spiffs|esp32.esp32.esp32cam|esp32-cam.bin|min_spiffs"
  "flamingo|esp32_flamingo|esp32:esp32:esp32:PartitionScheme=default|esp32.esp32.esp32|esp32-flamingo.bin|default"
  "default|esp32_default|esp32:esp32:esp32:PartitionScheme=default|esp32.esp32.esp32|esp32-default.bin|default"
)

should_build() {
  local id="$1"
  shift
  [[ $# -eq 0 ]] && return 0
  local target
  for target in "$@"; do
    [[ "$target" == "$id" ]] && return 0
  done
  return 1
}

build_project() {
  local id sketch_dir fqbn build_subdir ota_basename partition
  IFS='|' read -r id sketch_dir fqbn build_subdir ota_basename partition <<<"$1"

  local sketch="$ROOT/$sketch_dir"
  local build_path="$sketch/build/$build_subdir"
  local ino_name ota_name
  ino_name="$(basename "$sketch_dir").ino"
  ota_name="$(ota_filename "$ota_basename" "$sketch_dir")"

  if [[ ! -f "$sketch/secrets.h" ]]; then
    echo "[$id] пропуск: нет $sketch/secrets.h (скопируйте secrets.example.h)" >&2
    return 1
  fi

  echo "[$id] компиляция ($(read_fw_version "$sketch_dir"), $(date +%Y-%m-%d))..."
  "$ARDUINO_CLI" compile \
    --fqbn "$fqbn" \
    --build-path "$build_path" \
    --build-property "compiler.cpp.extra_flags=-I$ROOT/include" \
    "$sketch"

  local src_bin="$build_path/${ino_name}.bin"
  if [[ ! -f "$src_bin" ]]; then
    echo "[$id] ошибка: не найден $src_bin" >&2
    return 1
  fi

  mkdir -p "$OTA_DIR"
  cp "$src_bin" "$OTA_DIR/$ota_name"

  local size sha
  size="$(wc -c < "$OTA_DIR/$ota_name" | tr -d ' ')"
  sha="$(shasum -a 256 "$OTA_DIR/$ota_name" | awk '{print $1}')"
  echo "[$id] -> ota/$ota_name  (${size} bytes, sha256=${sha:0:12}...)"

  python3 "$ROOT/scripts/update-firmware-manifest.py" "$id" "$sketch_dir" "$(read_fw_version "$sketch_dir")" "$partition" "$ota_name" "$size" "$sha"
}

main() {
  local targets=("$@")
  local failed=0
  local built=0

  echo "arduino-cli: $("$ARDUINO_CLI" version)"
  echo "выходная папка: $OTA_DIR"
  echo

  local entry id
  for entry in "${PROJECTS[@]}"; do
    IFS='|' read -r id _ _ _ _ _ <<<"$entry"
    if [[ ${#targets[@]} -gt 0 ]] && ! should_build "$id" "${targets[@]}"; then
      continue
    fi
    if build_project "$entry"; then
      built=$((built + 1))
    else
      failed=$((failed + 1))
    fi
    echo
  done

  if [[ $built -eq 0 ]]; then
    echo "Ничего не собрано. Доступные цели: flat, balcony, cam, flamingo, default" >&2
    exit 1
  fi

  if [[ $failed -gt 0 ]]; then
    echo "Готово с ошибками: $built успешно, $failed с ошибками." >&2
    exit 1
  fi

  echo "Готово: $built бинарник(ов) в ota/"
  ls -lh "$OTA_DIR"/*.bin 2>/dev/null || true
}

main "$@"
