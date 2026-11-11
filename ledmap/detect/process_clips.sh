#!/bin/bash
# usage: run from ledmap/data: ../detect/process_clips.sh DIR clip.mov ...  -> cache_cNNN, lowres_cNNN, out_cNNN for each clip
D=$1; shift
HERE=$(cd "$(dirname "$0")" && pwd)
PY=$HERE/.venv/bin/python
for f in "$@"; do
  name=$(basename $f .mov); n=${name: -3}
  crop=$($PY $HERE/treebox.py $D/$name.mov)
  echo "== c$n crop $crop"
  [ -d cache_c$n ] || $PY $HERE/extract.py $D/$name.mov cache_c$n --crop $crop
  [ -d lowres_c$n ] || $PY $HERE/extract.py $D/$name.mov lowres_c$n --width 960
  $PY $HERE/decode.py cache_c$n codes_mA_s1_n4800.csv --symbol-ms 100 --lowres lowres_c$n --bg 20,150,330,540 \
    --cycles 3 --core-r 1.5 --ring 3,5 --dog 1.5,4 --peak-win 5 --blob-thresh 0.1 --norm-white --features ring \
    --kmeans-init --gap-min 6 --max-dist 3 -o out_c$n
done
