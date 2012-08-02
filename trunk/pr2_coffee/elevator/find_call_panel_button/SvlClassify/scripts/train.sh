#!/bin/bash

callDir=/home/stair/ellen/stair/lasik2/;
baseDir=/afs/cs.stanford.edu/group/stairperception/scratch/Data/elevator/;
modelsDir=${baseDir}/models/;
dataDir=${baseDir}/train_data/;
cacheDir=${baseDir}/caches/;

posFolder=$1;
negFolder=$2;
patchSize=$3;
dictTrain=$4;

saveFile=${modelsDir}/${posFolder}.dictionary.xml;
dictTrainDir=${dataDir}/${posFolder}/;
rm -f ${modelsDir}/${posFolder}.dictionary.xml;
rm -f ${modelsDir}/${posFolder}.model;

date;
if [ $dictTrain -gt 0 ] 
then
    newDictTrainDir=${dataDir}/${posFolder}_dict_train/;
    mkdir $newDictTrainDir;
    files=($dictTrainDir*.jpg);        # create an array of the files.
    N=${#files[@]};          # Number of members in the array
    for i in `seq 1 $dictTrain`; do
	((f=RANDOM%N));
	randomFile=${files[$f]};
	cp $randomFile $newDictTrainDir;
    done;
    dictTrainDir=$newDictTrainDir;
fi

date;
echo ${callDir}/bin/buildPatchDictionary -o $saveFile -n $patchSize $dictTrainDir 46 23;
${callDir}/bin/buildPatchDictionary -o $saveFile -n $patchSize $dictTrainDir 46 23;
date;
if [ $dictTrain -gt 0 ] 
then
    rm -Rf $dictTrainDir;
fi
echo
echo
echo ...CALLING TRAIN OBJECT DETECTOR...
echo
echo

echo ${callDir}/svl/scripts/trainObjectDetector.pl -i $modelsDir -b ${callDir}/bin -d ${dataDir} -C ${cacheDir} -n $patchSize -N $negFolder $posFolder
${callDir}/svl/scripts/trainObjectDetector.pl -i $modelsDir -b ${callDir}/bin -d ${dataDir} -C ${cacheDir} -n $patchSize -N $negFolder $posFolder
date;

echo DONE!