#!/usr/bin/env bash

set -euo pipefail

repository="${1:-.}"
cd "$repository"

failed=0

report_error() {
  printf 'Modelica library check: %s\n' "$*" >&2
  failed=1
}

while IFS= read -r package_file; do
  package_directory="${package_file%/package.mo}"
  child_count="$(find "$package_directory" -maxdepth 1 -type f -name '*.mo' ! -name package.mo | wc -l)"
  if [ "$child_count" -gt 0 ] && [ ! -f "$package_directory/package.order" ]; then
    report_error "$package_directory has $child_count child classes but no package.order"
  fi
done < <(find . -type f -name package.mo ! -path './.git/*' | sort)

while IFS= read -r order_file; do
  package_directory="${order_file%/package.order}"
  duplicates="$(sort "$order_file" | uniq -d)"
  if [ -n "$duplicates" ]; then
    report_error "$order_file contains duplicate entries: $duplicates"
  fi

  while IFS= read -r child_file; do
    child_name="${child_file##*/}"
    child_name="${child_name%.mo}"
    if ! rg -q "^${child_name}$" "$order_file"; then
      report_error "$order_file does not list $child_name"
    fi
  done < <(find "$package_directory" -maxdepth 1 -type f -name '*.mo' ! -name package.mo | sort)
done < <(find . -type f -name package.order ! -path './.git/*' | sort)

while IFS= read -r modelica_file; do
  relative="${modelica_file#./}"
  directory="${relative%/*}"
  filename="${relative##*/}"
  if [ "$filename" = package.mo ]; then
    if [[ "$directory" == */* ]]; then
      expected="${directory%/*}"
    else
      expected=""
    fi
  else
    expected="$directory"
  fi
  expected="${expected//\//.}"
  actual="$(sed -n '1{s/[[:space:]]//g;p;q;}' "$modelica_file")"
  if [ -z "$expected" ]; then
    if [ "$actual" != 'within;' ]; then
      report_error "$relative starts with '$actual'; expected 'within;'"
    fi
  elif [ "$actual" != "within${expected};" ]; then
    report_error "$relative starts with '$actual'; expected 'within ${expected};'"
  fi
done < <(find . -type f -name '*.mo' ! -path './.git/*' ! -path './tools/rumoca-repros/*' | sort)

if [ "$failed" -ne 0 ]; then
  exit 1
fi

printf 'Modelica library structure check passed.\n'
