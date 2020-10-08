#!/bin/bash
############################################################
# DFNEXT freeture video continuous capture script
############################################################

systemctl stop freeture.service

folderNameDate=$(date -u +"%Y_%m_%d")
fileNameTime=$(date -u +"%Y-%m-%dT%H%M%S")

baseFolder=/data0/video_frames
[ -d ${baseFolder} ] || mkdir ${baseFolder}

todayFolder=${baseFolder}/${folderNameDate}
[ -d ${todayFolder} ] || mkdir ${todayFolder}

logFolder=${baseFolder}/log
[ -d ${logFolder} ] || mkdir ${logFolder}

logFName=${logFolder}/${fileNameTime}_continuous_capture.log

dataFileName=$(hostname)_${folderNameDate}_allskyvideo_frame

runTimeSeconds=2
videoGain=29
expTime=33000
imgWidth=1080
imgHeight=1080
imgStartX=420
imgStartY=60

echo "freeture -m 2 --gain ${videoGain} --exposure ${expTime} --width ${imgWidth} --height ${imgHeight} --startx ${imgStartX} --starty ${imgStartY} --savepath ${todayFolder}/ --filename ${dataFileName} --fits -t ${runTimeSeconds} -c /usr/local/etc/dfn/freeture.cfg >> ${logFName}" >> ${logFName}
echo "=============================================" >> ${logFName}
freeture -m 2 --gain ${videoGain} --exposure ${expTime} --width ${imgWidth} --height ${imgHeight} --startx ${imgStartX} --starty ${imgStartY} --savepath ${todayFolder}/ --filename ${dataFileName} --fits -t ${runTimeSeconds} -c /usr/local/etc/dfn/freeture.cfg >> ${logFName}
