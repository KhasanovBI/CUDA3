#!/bin/bash
ls ./pos | awk '{print "./pos/" $0 " 1 0 0 100 100"}' > nums.info
ls ./neg | awk '{print "./neg/" $0 }' > bg.txt
opencv_createsamples -info nums.info -num 10 -w 24 -h 24 -vec nums.vec
rm -rf data
mkdir data
opencv_traincascade -data data -vec nums.vec -bg bg.txt -numPos 10 -numNeg 15 -numStages 20 -w 24 -h 24 -featureType LBP