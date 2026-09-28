#!/usr/bin/env bash
set -eu

cd "$(dirname "$0")"

latexmk -C >/dev/null 2>&1 || true

find . -maxdepth 2 \( \
  -name '*.aux' -o \
  -name '*.bbl' -o \
  -name '*.bcf' -o \
  -name '*.blg' -o \
  -name '*.fdb_latexmk' -o \
  -name '*.fls' -o \
  -name '*.lof' -o \
  -name '*.log' -o \
  -name '*.lot' -o \
  -name '*.out' -o \
  -name '*.pdf' -o \
  -name '*.run.xml' -o \
  -name '*.synctex.gz' -o \
  -name '*.toc' -o \
  -name '*.nav' -o \
  -name '*.snm' -o \
  -name '*.vrb' -o \
  -name '*.xdv' \
\) -delete

echo "LaTeX build artifacts removed."
