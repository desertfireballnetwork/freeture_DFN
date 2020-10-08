#!/bin/bash
############################################################
# DFNEXT freeture video continuous capture script
############################################################

systemctl stop freeture.service

folderNameDate=$(date -u +"%Y-%m-%d")
fileNameTime=$(date -u +"%Y-%m-%dT%H%M%S")

baseFolder=/data0/video_frames
[ -d ${baseFolder} ] || mkdir ${baseFolder}

todayFolder=${baseFolder}/${folderNameDate}
[ -d ${todayFolder} ] || mkdir ${todayFolder}

logFolder=${baseFolder}/log
[ -d ${logFolder} ] || mkdir ${logFolder}

logFName=${logFolder}/${fileNameTime}_continuous_capture.log

dataFileName=$(hostname)_${folderNameDate}_allskyvideo_frame_

echo "freeture -m 2 --gain 29 --exposure 33000 --width 1080 --height 1080 --startx 420 --starty 60 --savepath ${todayFolder}/ --filename ${dataFileName} --fits -t 1800 -c /usr/local/etc/dfn/freeture.cfg > ${logFName}" >> ${logFName}
echo "=============================================" >> ${logFName}
freeture -m 2 --gain 29 --exposure 33000 --width 1080 --height 1080 --startx 420 --starty 60 --savepath ${todayFolder}/ --filename ${dataFileName} --fits -t 1800 -c /usr/local/etc/dfn/freeture.cfg >> ${logFName}
