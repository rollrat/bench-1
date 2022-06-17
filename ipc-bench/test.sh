#!/bin/bash -i

result_directory=$(pgd)
output="$result_directory/output"

if [ ! -d $output ]; then
		mkdir $output
fi

cd ${result_directory%/ipc-bench*}/build/source
echo ${result_directory%/ipc-bench*}/build/source

technologies=(
		# shm
		# mq
		# domain
		# fifo
		# pipe
		# mmap
		# tcp
		# signal
		zeromq
)

if [ $(uname) = Linux ]; then
		technologies+=( eventfd-uni )
fi

i=0
while [ "$i" -lt 100 ]
do
  for tech in "${technologies[@]}"; do
  		echo "Running $tech ..."
  		cd $tech

  		if [ -f output/$tech.result ]; then
  				rm $output/$tech.result
  		fi

  		for size_power in $(seq 0 3); do
  				size=$((10**size_power))
  				for count_power in $(seq 0 6); do
  						count=$((10**count_power))
  						./$tech -s $size -c $count >> "$output/$tech.result"
              # echo ./$tech -s $size -c $count
  						if [ $tech = zeromq ]; then
  								sleep 0.2
  						else
  								sleep 0.1
  						fi
  				done
  		done

  		# Clean up, just in case
  		killall -9 "$tech-server" &> /dev/null
  		killall -9 "$tech-client" &> /dev/null

  		cd ..
  done
  i=$(expr $i + 1)
  echo loop ${i}
done

cd $result_directory
