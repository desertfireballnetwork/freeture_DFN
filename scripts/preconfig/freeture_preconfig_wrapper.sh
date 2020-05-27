#!/bin/bash

FREETURE_LOG_DIR="/data0/log/freeture"
FREETURE_PRECONFIG_LOG_DIR=${FREETURE_LOG_DIR}/preconfig
[ -d ${FREETURE_PRECONFIG_LOG_DIR} ] || mkdir -p ${FREETURE_PRECONFIG_LOG_DIR} 
FREETURE_PRECONFIG_LOG=${FREETURE_PRECONFIG_LOG_DIR}/freeture_preconfig_$(date -u "+%Y-%m-%d_%H%M%S")UTC.log

FREETURE_DEAMON_CFG="/usr/local/etc/dfn/freeture.cfg"

preconfig()
{
    echo ""
    echo "Generating freeture config file..."
    python3 /opt/dfn-software/freeture_preconfig.py > ${FREETURE_PRECONFIG_LOG} 2>&1
    preconfig_result=${?}
    # preconfig_result  is  0 if no python err, otherwise non-zero
    cat ${FREETURE_PRECONFIG_LOG}
    if [ ${preconfig_result} -eq 0 ]
    then
	# modification of freeture config succeeded, let's use this one
	freetureconf=$(tail -n 1 ${FREETURE_PRECONFIG_LOG})
	echo "Generating config file OK as ${freetureconf}"
	# dump it as default for future, if something went wrong
	echo save as default freeture config file ${DEAMON_CFG}
	cp ${freetureconf} ${FREETURE_DEAMON_CFG}
    else
	# running of python3 /opt/dfn-software/freeture_preconfig.py failed
	# perhaps some internal interval_control_sw incompatibility
	# let's go with default ${DEAMON_CFG}
	echo "Generating config file FAILED, using previous one in file ${DEAMON_CFG}"
    fi
}

preconfig

