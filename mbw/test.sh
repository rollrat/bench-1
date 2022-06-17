#!/bin/bash
i=0

target_size=(1024 2048 4096 6120 8192)
block_size=(1 32 64 128 256 512 1024 1536 2048 3072 4096 6120 8192 12288 16384 20480 25600 31200 32768 40000 45000 50000 65000 65536)

cnt=0
total_cnt=$((${#block_size[@]} * ${#target_size[@]}))

while [ "$i" -lt 20 ]
do
  for tsize in "${target_size[@]}"; do
    for bsize in "${block_size[@]}"; do
      ./mbw ${tsize} -b ${bsize} -q >> result/${i}.${tsize}.${bsize}.mbw
      # ./mbw-release ${tsize} -b ${bsize} -q >> result/${i}.${tsize}.${bsize}.release
      cnt=$(expr $cnt + 1)
      echo ${cnt}/${total_cnt}
    done
  done
  echo loop ${i}
  i=$(expr $i + 1)
done