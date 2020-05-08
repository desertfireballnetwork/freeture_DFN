#!/usr/bin/python
# 
# This file is part of the Desert Fireball Network camera control system.
# It creates an up to date config file for freeture (https://github.com/desertfireballnetwork/freeture_DFN)
# Notes:

# freetureconf=$(python3 /opt/dfn-software/freeture_preconfig.py | tail -n 1)
# /usr/local/bin/freeture -m 3 -c ${freetureconf}


from __future__ import print_function, division

#import logging
import os
import sys
import shutil
import datetime

# local imports
import dfn_functions as dfn
import leostick as micro
import config_handler as cfg



INSTRUMENT_TYPE = 'allskyvideo'
prog_dir = os.path.dirname(os.path.abspath(__file__))
FREETURE_CFG_FILE_TEMPLATE = os.path.join(prog_dir, r'dfn_freeture_template.cfg')


#STATION_NAME = DFNEXT009
#TELESCOP = test_lab
#INSTRUME = allskyvideo
#SITELONG = 115.89
#SITELAT = -32.0
#SITEELEV = 50.0

#[station]
#gps_lock = N
#location = test_lab
#lat = -32.00720
#altitude = 50.0
#hostname = DFNEXT027
#lon = 115.89469


DFN_FREETURE_CONFIG_TRANSLATOR = {
                                  'STATION_NAME' : 'location',
                                  'TELESCOP' : 'hostname',
                                  'SITELONG' : 'lon',
                                  'SITELAT' : 'lat',
                                  'SITEELEV' : 'altitude',
                                  'GPS_LOCK' : 'gps_lock',
                                 }

def prepare_intrument_data_folder(instrument=INSTRUMENT_TYPE):
    """
    
    """
    
    # Get program directory.
    prog_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Load new config file from master copy in /opt/dfn-software
    config_file = os.path.join(prog_dir, r'dfnstation.cfg')
    config_dict = cfg.load_config(config_file)
    config_dict['internal']['config_file'] = config_file

    # Setup new data path for this instrument session
    data_path = dfn.make_data_path(config_dict['internal']['data_directory'],
                                   secs=False,
                                   instrument=instrument)
    
    
    # try to update config file from GPS
    try:
        # Initialise microcontroller
        ser = micro.connect_to_leostick()
        
        # Get new gps location, if available and report lock
        (config_dict['station']['lon'],
        config_dict['station']['lat'],
        config_dict['station']['altitude'],
        config_dict['station']['gps_lock']) = micro.update_GPS_location(
                                            config_dict['station']['lon'],
                                            config_dict['station']['lat'],
                                            config_dict['station']['altitude'])
        
        ser.close()
    except (Exception, SystemExit) as e:
        print('Problem talking to microcontroller, using config values from default file', file=sys.stderr)
        config_dict['station']['gps_lock'] = 'N'
        
        
     
     
     
    # Save a local copy of the config file
    session_config_fname = os.path.join(data_path, 'dfnstation.cfg')
    #cfg.save_config_file(session_config_fname, config_dict)
        
    return data_path, session_config_fname, config_dict


def make_freeture_config(dfn_config_dic,
                         freeture_cfg_template,
                         destination_folder):
    
    
    freeture_session_file = os.path.join(destination_folder, (dfn_config_dic['station']['hostname']
                                                                + '_'
                                                                + datetime.datetime.utcnow().strftime('%Y-%m-%d_%H%M%S')
                                                                + '_'
                                                                + 'freeture.cfg'))
    
    
    if os.path.isfile(freeture_session_file):
        shutil.remove(freeture_session_file)
    shutil.copyfile(freeture_cfg_template, freeture_session_file)
    
    
    for freet_k, dfn_k in DFN_FREETURE_CONFIG_TRANSLATOR.items():
        
        comm = ("sed -i "
                + "\'s/^"
                + str(freet_k)
                + " *= *.*/"
                + str(freet_k)
                + " = "
                + str(dfn_config_dic['station'][dfn_k])
                + "/\' "
                + freeture_session_file)
        
        os.system(comm)
    
    
    
    return freeture_session_file
    

def main():
    
    data_path, config_file, dfn_config_dic = prepare_intrument_data_folder(instrument='allskyvideo')
    
    freeture_session_file = make_freeture_config(dfn_config_dic=dfn_config_dic,
                                                 freeture_cfg_template=FREETURE_CFG_FILE_TEMPLATE,
                                                 destination_folder=data_path)
    
    print(freeture_session_file)
    return freeture_session_file


if __name__ == '__main__':
    main()
