#!/bin/bash

if [[ ( $@ == "--help") ||  $@ == "-h" ]]
then 
	echo "Usage: $0 [IMAGE_NAME]"
	#exit 0
else
	if [[ ($# -eq 0 ) ]]
	then 
		echo "Usage: $0 [IMAGE_NAME]"
		#exit 0
	else
		docker build -t $1 --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .
	fi 
fi 