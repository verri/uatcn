#!/bin/bash

parallel () {
  while [ "$(jobs | wc -l)" -ge $(nproc) ]
  do
    wait -n
  done
  echo "$@"
  "$@" &
}

cmake --build build-release

trap 'kill $(jobs -p); exit 1' SIGINT SIGTERM SIGKILL

maxtime=1000
for seed in {1..20}; do
  for lambda in $(seq 40 -2 16) {15..1}; do
    for space in "35,35,5" "25,25,3" "15,15,5" "15,15,3" "15,15,2" "15,15,1"; do
      filename="data/agents,d=$space,s=$seed,l=$lambda.csv.gz"
      if [ -f "$filename" ]; then
        echo "Skipping s=$seed λ=$lambda"
      else
        nfilename="data/network,d=$space,s=$seed,l=$lambda,t={time}.csv.gz"
        parallel build-release/simulation -d $(echo $space | sed 's/,/ /g') -s $seed -l $lambda \
          -t $maxtime -p $((maxtime - 5 + 1)) \
          -n "$nfilename" \
          -o "$filename"
      fi
    done
  done
done

wait
