callDir=/home/acoates/lasik/;

saveDir=$1;
modelDir=$2;
model=$3;
saveName=$4;
saveFile=${saveDir}/${saveName}.xml;
testFile=$5;
height=$6;
width=$7;

scale=$8;

deltaX=10;
deltaY=10;
resizeHeight=`echo "scale=3; $height / $scale" | bc`;
resizeHeight=`echo $resizeHeight / 1 | bc`;
resizeWidth=`echo "scale=3; $width / $scale" | bc`;
resizeWidth=`echo $resizeWidth / 1 | bc`;

date;
echo $resizeHeight

echo ${callDir}/bin/classifyImages -set svlVision.svlSlidingWindowDetector deltaX $deltaX -set svlVision.svlSlidingWindowDetector deltaY $deltaY -set svlVision.svlImageLoader resizeWidth $resizeWidth -set svlVision.svlImageLoader resizeHeight $resizeHeight -o $saveFile ${modelDir}/${model}.dictionary.xml ${modelDir}/${model}.model $testFile

${callDir}/bin/classifyImages -set svlVision.svlSlidingWindowDetector deltaX $deltaX -set svlVision.svlSlidingWindowDetector deltaY $deltaY -set svlVision.svlImageLoader resizeWidth $resizeWidth -set svlVision.svlImageLoader resizeHeight $resizeHeight -o $saveFile ${modelDir}/${model}.dictionary.xml ${modelDir}/${model}.model $testFile

echo
date
echo ${callDir}/bin/scaleDetections -s $scale $saveFile $saveFile

${callDir}/bin/scaleDetections -s $scale $saveFile $saveFile

echo
date
