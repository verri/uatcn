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

maxtime=500
for lambda in {30..1}; do
  for seed in {1..20}; do
    filename="data/agents,d=35,35,5,150,s=$seed,l=$lambda.csv.gz"
    if [ -f "$filename" ]; then
      echo "Skipping s=$seed λ=$lambda"
    else
      nfilename="data/network,d=35,35,5,150,s=$seed,l=$lambda,t={time}.csv.gz"
      parallel build-release/simulation -d 35 35 5 150 -s $seed -l $lambda \
        -t $maxtime -p $((maxtime - 5 + 1)) \
        -n "$nfilename" \
        -o "$filename"
    fi
  done
done

wait
