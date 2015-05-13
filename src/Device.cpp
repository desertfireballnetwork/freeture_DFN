/*
								Device.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*	This file is part of:	freeture
*
*	Copyright:		(C) 2014-2015 Yoan Audureau
*                               FRIPON-GEOPS-UPSUD-CNRS
*
*	License:		GNU General Public License
*
*	FreeTure is free software: you can redistribute it and/or modify
*	it under the terms of the GNU General Public License as published by
*	the Free Software Foundation, either version 3 of the License, or
*	(at your option) any later version.
*	FreeTure is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*	GNU General Public License for more details.
*	You should have received a copy of the GNU General Public License
*	along with FreeTure. If not, see <http://www.gnu.org/licenses/>.
*
*	Last modified:		20/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Device.cpp
* \author  Yoan Audureau -- FRIPON-GEOPS-UPSUD
* \version 1.0
* \date    02/09/2014
* \brief
*/

#include "Device.h"

boost::log::sources::severity_logger< LogSeverityLevel >  Device::logger;
Device::_Init Device::_initializer;

Device::Device(CamType type){

    STATION_NAME = "STATION";
    DATA_PATH = "./";

    switch(type){

        case BASLER_GIGE :

            {
                #ifdef USE_PYLON
                    BOOST_LOG_SEV(logger, normal) << "INPUT : BASLER_GIGE -> Use Pylon";
                    cam = new CameraGigeSdkPylon();
                #else
                    #ifdef LINUX
                        BOOST_LOG_SEV(logger, normal) << "INPUT : BASLER_GIGE -> Use Aravis";
                        cam = new CameraGigeSdkAravis();
                    #endif
                #endif
            }

            break;

        case DMK_GIGE:

            {

                #ifdef WINDOWS
                    BOOST_LOG_SEV(logger, normal) << "INPUT : DMK_GIGE -> Use Imaging Source";
                    cam = new CameraGigeSdkIc();
                #else
                    #ifdef LINUX
                        BOOST_LOG_SEV(logger, normal) << "INPUT : DMK_GIGE -> Use Aravis";
                        cam = new CameraGigeSdkAravis(true);
                    #endif
                #endif

            }

            break;

        default :

            cam = NULL;

    }
}

Device::~Device(){

    if(cam != NULL) delete cam;
}

bool Device::prepareDevice(CamType type, string cfgFile){

	try{

        Configuration cfg;
		cfg.Load(cfgFile);

		switch(type){

			case FRAMES :

				{

                    // Get frames location.
					string INPUT_FRAMES_DIRECTORY_PATH; cfg.Get("INPUT_FRAMES_DIRECTORY_PATH", INPUT_FRAMES_DIRECTORY_PATH);
					BOOST_LOG_SEV(logger, normal) << "Read INPUT_FRAMES_DIRECTORY_PATH from configuration file : " << INPUT_FRAMES_DIRECTORY_PATH;

					// Get separator position in frame's name.
					int FRAMES_SEPARATOR_POSITION; cfg.Get("FRAMES_SEPARATOR_POSITION", FRAMES_SEPARATOR_POSITION);
					BOOST_LOG_SEV(logger, normal) << "Read FRAMES_SEPARATOR_POSITION from configuration file : " << FRAMES_SEPARATOR_POSITION;

                    // Create camera using pre-recorded fits2D in input.
					cam = new CameraFrames(INPUT_FRAMES_DIRECTORY_PATH, FRAMES_SEPARATOR_POSITION);
					cam->grabStart();

				}

				break;

			case VIDEO:

				{
				    // Get frames locations.
					string	INPUT_VIDEO_PATH; cfg.Get("INPUT_VIDEO_PATH", INPUT_VIDEO_PATH);

					vector<string> videoList;

                    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                    boost::char_separator<char> sep(",");
                    tokenizer tokens(INPUT_VIDEO_PATH, sep);

                    int n = 1;

                    for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
                        videoList.push_back(*tok_iter);
                        cout << "VIDEO " << Conversion::intToString(n) << " : " << *tok_iter << endl;
                        n++;
                    }

                    // Create camera using pre-recorded video in input.
					cam = new CameraVideo(videoList);
					cam->grabStart();
				}

				break;

			default :

                // Get camera ID to use.
                cfg.Get("CAMERA_ID", CAMERA_ID);
                BOOST_LOG_SEV(logger, notification) << "CAMERA_ID : " << CAMERA_ID;

                // Get data location.
                cfg.Get("DATA_PATH", DATA_PATH);
                BOOST_LOG_SEV(logger, notification) << "DATA_PATH : " << DATA_PATH;

                // Get station name.
                cfg.Get("STATION_NAME", STATION_NAME);
                BOOST_LOG_SEV(logger, notification) << "STATION_NAME : " << STATION_NAME;

                // Get acquisition format.
                string acq_bit_depth; cfg.Get("ACQ_BIT_DEPTH", acq_bit_depth);
                EParser<CamBitDepth> cam_bit_depth;
                ACQ_BIT_DEPTH = cam_bit_depth.parseEnum("ACQ_BIT_DEPTH", acq_bit_depth);
                BOOST_LOG_SEV(logger, notification) << "ACQ_BIT_DEPTH : " << acq_bit_depth;

                // Get night exposure time.
                cfg.Get("ACQ_NIGHT_EXPOSURE", ACQ_EXPOSURE);
                BOOST_LOG_SEV(logger, notification) << "ACQ_NIGHT_EXPOSURE : " << ACQ_EXPOSURE;

                // Get night gain.
                cfg.Get("ACQ_NIGHT_GAIN", ACQ_GAIN);
                BOOST_LOG_SEV(logger, notification) << "ACQ_NIGHT_GAIN : " << ACQ_GAIN;

                // Get sunrise time.
                string sunrise_time;
                cfg.Get("SUNRISE_TIME", sunrise_time);
                BOOST_LOG_SEV(logger, notification) << "SUNRISE_TIME : " << sunrise_time;

                // Get acquisition FPS.
                cfg.Get("ACQ_FPS", ACQ_FPS);
                BOOST_LOG_SEV(logger, notification) << "ACQ_FPS : " << ACQ_FPS;

                // Get exposure control save infos option.
                cfg.Get("EXPOSURE_CONTROL_SAVE_INFOS", EXPOSURE_CONTROL_SAVE_INFOS);
                BOOST_LOG_SEV(logger, notification) << "EXPOSURE_CONTROL_SAVE_INFOS : " << EXPOSURE_CONTROL_SAVE_INFOS;

                // Get exposure control save image option.
                cfg.Get("EXPOSURE_CONTROL_SAVE_IMAGE", EXPOSURE_CONTROL_SAVE_IMAGE);
                BOOST_LOG_SEV(logger, notification) << "EXPOSURE_CONTROL_SAVE_IMAGE : " << EXPOSURE_CONTROL_SAVE_IMAGE;

                // Get exposure control option.
                cfg.Get("EXPOSURE_CONTROL_ENABLED", EXPOSURE_CONTROL_ENABLED);
                BOOST_LOG_SEV(logger, notification) << "EXPOSURE_CONTROL_ENABLED : " << EXPOSURE_CONTROL_ENABLED;

                cfg.Get("EXPOSURE_CONTROL_FREQUENCY", EXPOSURE_CONTROL_FREQUENCY);
                BOOST_LOG_SEV(logger, notification) << "EXPOSURE_CONTROL_FREQUENCY : " << EXPOSURE_CONTROL_FREQUENCY;

                // Get schedule option status.
                cfg.Get("ACQ_SCHEDULE_ENABLED", ACQ_SCHEDULE_ENABLED);
                BOOST_LOG_SEV(logger, notification) << "ACQ_SCHEDULE_ENABLED : " << ACQ_SCHEDULE_ENABLED;

                if(ACQ_SCHEDULE_ENABLED){

                    string	sACQ_SCHEDULE;
                    cfg.Get("ACQ_SCHEDULE", sACQ_SCHEDULE);

                    vector<string> sch1;

                    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                    boost::char_separator<char> sep(",");
                    tokenizer tokens(sACQ_SCHEDULE, sep);

                    int n = 1;
                    BOOST_LOG_SEV(logger, notification) << "SCHEDULE : ";
                    for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
                        string s = *tok_iter;
                        std::transform(s.begin(), s.end(),s.begin(), ::toupper);
                        sch1.push_back(s);
                        cout << "-> " << Conversion::intToString(n) << " - " << s << endl;
                        BOOST_LOG_SEV(logger, notification) << "-> " << Conversion::intToString(n) << " - " << s;
                        n++;
                    }

                    //23h25m10000000e400g12f1n
                    for(int i = 0; i < sch1.size(); i++){

                         typedef boost::tokenizer<boost::char_separator<char> > tokenizer_;
                         boost::char_separator<char> sep_("HMEGFN");
                         tokenizer tokens_(sch1.at(i), sep_);

                         vector<string> sp;

                         for(tokenizer::iterator tok_iter_ = tokens_.begin();tok_iter_ != tokens_.end(); ++tok_iter_){

                            sp.push_back(*tok_iter_);

                         }

                         if(sp.size() == 6){

                            AcqRegular r = AcqRegular(atoi(sp.at(0).c_str()), atoi(sp.at(1).c_str()), atoi(sp.at(2).c_str()), atoi(sp.at(3).c_str()), atoi(sp.at(4).c_str()), atoi(sp.at(5).c_str()));
                            ACQ_SCHEDULE.push_back(r);

                         }

                    }

                }


                // Get use mask option.
                cfg.Get("ACQ_MASK_ENABLED", ACQ_MASK_ENABLED);
                BOOST_LOG_SEV(logger, notification) << "ACQ_MASK_ENABLED : " << ACQ_MASK_ENABLED;

                if(ACQ_MASK_ENABLED){

                    cfg.Get("ACQ_MASK_PATH", ACQ_MASK_PATH);
                    BOOST_LOG_SEV(logger, notification) << "ACQ_MASK_PATH : " << ACQ_MASK_PATH;

                    ACQ_MASK = imread(ACQ_MASK_PATH, CV_LOAD_IMAGE_GRAYSCALE);

                    if(!ACQ_MASK.data){

                        BOOST_LOG_SEV(logger, fail) << " Can't load the mask from this location : " << ACQ_MASK_PATH;
                        throw "Failed to load the mask";

                    }

                }

                BOOST_LOG_SEV(logger, notification) << "Loading fits keywords...";
                fitsHeader.loadKeywordsFromConfigFile(cfgFile);

                runContinuousAcquisition();

		}

	}catch(exception& e){

		cout << e.what() << endl;
		return false;

	}catch(const char * msg){

		cout << msg << endl;
		return false;

	}

	return true;
}


void Device::controlExposureTime(float msv){

    /*int currExp = cam->getExposureTime();
    cout << "currExp : " << currExp << endl;

    int expMin = -1, expMax = -1;
    cam->getExposureBounds(expMin,expMax);

    if(expMin!=-1 && expMax!=-1){

        if(msv > 2.4 && msv < 2.5){

            cout << "msv is correct. No need to change exposure time" << endl;

        }else if(msv < 2.4 && currExp == expMax){

            cout << "msv is not correct but impossible to increase exposure time because the value is already the maximum"

        }else if(msv > 2.5 && currExp == expMin){

            cout << "msv is not correct but impossible to decrease exposure time because the value is already to the minimum"

        }else if(msv < 2.4 && currExp != expMax){



            cout << "msv is not correct but impossible to increase exposure time because the value is already the maximum"

        }
    }
*/





}





void Device::runContinuousAcquisition(){

    /// List Gige Camera to check the ID.
    BOOST_LOG_SEV(logger, notification) << "Printing Connected Gige Camera...";
    cam->listGigeCameras();

    /// Create camera according to its ID.
    BOOST_LOG_SEV(logger, notification) << "Creating Device according ID " << CAMERA_ID << " ...";
    if(!cam->createDevice(CAMERA_ID))
        throw "Fail to create device.";

    /// Set camera format.
    BOOST_LOG_SEV(logger, notification) << "Setting acquisition format...";
    if(!cam->setPixelFormat(ACQ_BIT_DEPTH))
        throw "Fail to set Format.";

    /// Set camera exposure time.
    BOOST_LOG_SEV(logger, notification) << "Setting exposure time...";
    if(!cam->setExposureTime(ACQ_EXPOSURE))
        throw "Fail to set Exposure.";

    /// Set camera gain.
    BOOST_LOG_SEV(logger, notification) << "Setting gain...";
    if(!cam->setGain(ACQ_GAIN))
        throw "Fail to set Gain.";

    // Get exposure time bounds.
    cam->getExposureBounds(minExposureTime, maxExposureTime);

    cout << "maxExposureTime : " << maxExposureTime <<  endl;
    cout << "minExposureTime : " << minExposureTime <<  endl;

    // Get gain bounds.
    cam->getGainBounds(minGain, maxGain);


    /// Set camera fps.
    BOOST_LOG_SEV(logger, notification) << "Setting fps...";
    if(!cam->setFPS(ACQ_FPS))
        throw "Fail to set Fps.";

    /// Prepare grabbing.
    BOOST_LOG_SEV(logger, notification) << "Preparing camera to continuous acquisition...";
    if(!cam->grabStart())
        throw "Fail to start grab.";

    /// Start acquisition.
    BOOST_LOG_SEV(logger, notification) << "Starting acquisition...";
    cam->acqStart();

}

bool Device::loadDataset(){

    return cam->loadData();

}

bool Device::grabTest(){

    cam->grabTest();

}

void Device::listGigeCameras(){
	cam->listGigeCameras();
}

void Device::grabStop(){
	cam->grabStop();
}

bool Device::getDeviceStopStatus(){
	return cam->getStopStatus();
}

bool Device::getDatasetStatus(){
	return cam->getDataStatus();
}

void Device::acqStop(){
	cam->acqStop();
}

void Device::acqRestart(){
	runContinuousAcquisition();
}

bool Device::grabImage(Frame& newFrame){
	return cam->grabImage(newFrame);
}

bool Device::grabSingleImage(Frame &frame, int camID){
	return cam->grabSingleImage(frame, camID);
}

void Device::getExposureBounds(int &gMin, int &gMax){
	cam->getExposureBounds(gMin, gMax);
}

void Device::getGainBounds(int &eMin, int &eMax){
	cam->getGainBounds(eMin, eMax);
}

bool Device::getPixelFormat(CamBitDepth &format){
	return cam->getPixelFormat(format);
}

int Device::getWidth(){
	return cam->getWidth();
}

int Device::getHeight(){
	return cam->getHeight();
}

int Device::getFPS(){
    return cam->getFPS();
}

string Device::getModelName(){
	return cam->getModelName();
}

bool Device::setExposureTime(int exp){
	return cam->setExposureTime(exp);
}

int Device::getExposureTime(){
	return cam->getExposureTime();
}

bool Device::setGain(int gain){
	return cam->setGain(gain);
}

bool Device::setFPS(int fps){
	return cam->setFPS(fps);
}

bool Device::setPixelFormat(CamBitDepth depth){
	return cam->setPixelFormat(depth);
}
