#!/usr/bin/env bash
set -euo pipefail

cache_dir=${1:?usage: cache-rumoca-runtime.sh CACHE_DIRECTORY}
archive="$cache_dir/rumoca-runtime.export"
upstream_manifest="$cache_dir/upstream-runtime-paths"
cli_path=$(nix eval --raw .#rumoca-cli.outPath)
python_path=$(nix eval --raw .#rumoca-python-runtime.outPath)
expected_path=$(nix eval --raw .#rumoca-runtime.outPath)

for store_path in "$cli_path" "$python_path" "$expected_path"; do
  case "$store_path" in
    /nix/store/*-rumoca-*) ;;
    *)
      printf 'error: unexpected Rumoca output path: %s\n' "$store_path" >&2
      exit 2
      ;;
  esac
done

mkdir -p "$cache_dir"

if [ -s "$archive" ]; then
  if [ ! -s "$upstream_manifest" ]; then
    printf 'error: cached Rumoca runtime has no upstream-path manifest\n' >&2
    exit 2
  fi
  printf 'Importing cached Rumoca runtime %s\n' "$expected_path"
  mapfile -t upstream_paths < "$upstream_manifest"
  nix-store --realise "${upstream_paths[@]}"
  nix-store --import < "$archive"
else
  printf 'No cached Rumoca runtime; building %s\n' "$expected_path"
  built_path=$(nix build --no-link --print-out-paths .#rumoca-runtime)
  if [ "$built_path" != "$expected_path" ]; then
    printf 'error: built Rumoca path %s, expected %s\n' \
      "$built_path" "$expected_path" >&2
    exit 2
  fi

  cached_paths=("$python_path" "$cli_path" "$built_path")
  nix-store --query --requisites "$built_path" \
    | grep -Fvx \
      -e "$python_path" \
      -e "$cli_path" \
      -e "$built_path" \
    > "$upstream_manifest"

  temporary_archive="$archive.tmp"
  nix-store --export "${cached_paths[@]}" > "$temporary_archive"
  mv "$temporary_archive" "$archive"
  printf 'Prepared targeted Rumoca runtime cache (%s paths, %s bytes)\n' \
    "${#cached_paths[@]}" "$(wc -c < "$archive")"
fi

nix-store --verify-path "$expected_path"
test -x "$expected_path/bin/rumoca"
"$expected_path/bin/rumoca" --version
