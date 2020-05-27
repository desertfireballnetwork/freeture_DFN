/*
                        CameraUsbAravis.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau
*                               FRIPON-GEOPS-UPSUD-CNRS
*                   (C) 2018-2019 Martin Cupak -- DFN/GFO
*
*   License:        GNU General Public License
*
*   FreeTure is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*   FreeTure is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*   You should have received a copy of the GNU General Public License
*   along with FreeTure. If not, see <http://www.gnu.org/licenses/>.
*
*   Last modified:      14/08/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CameraUsbAravis.cpp
* \author  Yoan Audureau -- FRIPON-GEOPS-UPSUD
* \version 1.0
* \date    07/08/2018
* \brief   Use Aravis library to pilot USB Cameras.
*          https://wiki.gnome.org/action/show/Projects/Aravis?action=show&redirect=Aravis
*/

#include "CameraUsbAravis.h"

#ifdef LINUX

    boost::log::sources::severity_logger< LogSeverityLevel >  CameraUsbAravis::logger;
    CameraUsbAravis::Init CameraUsbAravis::initializer;

    CameraUsbAravis::CameraUsbAravis(bool shift):
    camera(NULL), mStartX(0), mStartY(0), mWidth(0), mHeight(0), fps(0), gainMin(0.0), gainMax(0.0),
    payload(0), exposureMin(0), exposureMax(0), gain(0), exp(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(shift), stream(NULL) {
        mExposureAvailable = true;
        mGainAvailable = true;
        mInputDeviceType = CAMERA;
    }

    CameraUsbAravis::CameraUsbAravis():
    camera(NULL), mStartX(0), mStartY(0), mWidth(0), mHeight(0), fps(0), gainMin(0.0), gainMax(0.0),
    payload(0), exposureMin(0), exposureMax(0), gain(0), exp(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(false), stream(NULL) {
        mExposureAvailable = true;
        mGainAvailable = true;
        mInputDeviceType = CAMERA;
    }

    CameraUsbAravis::~CameraUsbAravis(){

        if(stream != NULL)
            g_object_unref(stream);

        if(camera != NULL)
            g_object_unref(camera);

    }

    vector<pair<int,string>> CameraUsbAravis::getCamerasList() {

        vector<pair<int,string>> camerasList;

        ArvInterface *interface;

        //arv_update_device_list();

        int ni = arv_get_n_interfaces();


        for (int j = 0; j< ni; j++){

            const char* name = arv_get_interface_id (j);
            if (strcmp(name,"USB3Vision") == 0) {
                interface = arv_uv_interface_get_instance();
                arv_interface_update_device_list(interface);
                //int nb = arv_get_n_devices();

                int nb = arv_interface_get_n_devices(interface);

                for(int i = 0; i < nb; i++){

                    pair<int,string> c;
                    c.first = i;
                    //const char* str = arv_get_device_id(i);
                    const char* str = arv_interface_get_device_id(interface,i);
		    //const char* addr = arv_interface_get_device_address(interface,i);
		    const char* phyid = arv_interface_get_device_physical_id(interface,i);
                    std::string s = str;
                    c.second = "NAME[" + s + "] SDK[ARAVIS_USB] USB bus id: " + phyid;
                    camerasList.push_back(c);
                }
            }
        }

       return camerasList;

    }

    bool CameraUsbAravis::listCameras(){

        ArvInterface *interface;
        //arv_update_device_list();

        int ni = arv_get_n_interfaces ();

        cout << endl << "------------ USB CAMERAS WITH ARAVIS ----------" << endl << endl;

        for (int j = 0; j< ni; j++){

            interface = arv_uv_interface_get_instance();
            arv_interface_update_device_list(interface);
            //int nb = arv_get_n_devices();

            int nb = arv_interface_get_n_devices(interface);
            for(int i = 0; i < nb; i++){

                cout << "-> [" << i << "] " << arv_interface_get_device_id(interface,i)<< endl;
                //cout << "-> [" << i << "] " << arv_get_device_id(i)<< endl;

            }

            if(nb == 0) {
                cout << "-> No cameras detected..." << endl;
                return false;
            }
        }
        cout << endl << "------------------------------------------------" << endl << endl;

        return true;

    }

    bool CameraUsbAravis::createDevice(int id){
        
        string deviceName;

        if(!getDeviceNameById(id, deviceName))
          return false;

        camera = arv_camera_new(deviceName.c_str());	

        if(camera == NULL){

            BOOST_LOG_SEV(logger, fail) << "Fail to connect the camera.";
            return false;

        }

        return true;
    }

    bool CameraUsbAravis::setSize(int startx, int starty, int width, int height, bool customSize) {

        if(customSize) {

            arv_camera_set_region(camera, startx, starty, width, height);
            arv_camera_get_region (camera, &mStartX, &mStartY, &mWidth, &mHeight);
            BOOST_LOG_SEV(logger, notification) << "Camera region size : " << mWidth << "x" << mHeight;
            if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                BOOST_LOG_SEV(logger, notification) << "Starting from : " << mStartX << "," << mStartY;
            } else {
                BOOST_LOG_SEV(logger, warning) << "OffsetX, OffsetY are not available: cannot set offset.";
            }

        // Default is maximum size
        }else {

            int sensor_width, sensor_height;

            arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height);
            BOOST_LOG_SEV(logger, notification) << "Camera sensor size : " << sensor_width << "x" << sensor_height;

            arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height);
            arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight);

        }

        return true;

    }

    bool CameraUsbAravis::getDeviceNameById(int id, string &device){

        arv_update_device_list();

        int n_devices = arv_get_n_devices();

        for(int i = 0; i< n_devices; i++){

            if(id == i){

                device = arv_get_device_id(i);
                return true;

            }
        }

        BOOST_LOG_SEV(logger, fail) << "Fail to retrieve camera with this ID.";
        return false;

    }

    bool CameraUsbAravis::grabInitialization(){

      BOOST_LOG_SEV(logger, notification) << "CameraUsbAravis::grabInitialization()";
      //saveGenicamXml("./");

      guint bandwidth;
      gboolean isUV, isGV, isCam;
      const char * firmwareVer;

      // MC: need to get ArvDevice to be get more complete access to camera features
      ArvDevice *ad;
      ad = arv_camera_get_device( camera );
      
      bandwidth = arv_device_get_integer_feature_value (ad, "DeviceLinkThroughputLimit");
      BOOST_LOG_SEV(logger, notification) << "arv device Camera USB bandwidth limit : " << bandwidth;
      //      std::cout << "arv device Camera USB bandwidth limit : " << bandwidth;

      firmwareVer = arv_device_get_string_feature_value (ad, "DeviceFirmwareVersion");
      BOOST_LOG_SEV(logger, notification) << "arv device Camera Firmware Version : " << firmwareVer;

      // MC: 
      //double pwrVoltage = arv_device_get_float_feature_value( ad, "PowerSupplyVoltage" );
      //BOOST_LOG_SEV(logger, notification) << "arv device PowerSupplyVoltage : " << pwrVoltage;
      //double pwrCurrent = arv_device_get_float_feature_value( ad, "PowerSupplyCurrent" );
      //BOOST_LOG_SEV(logger, notification) << "arv device PowerSupplyCurrent : " << pwrVoltage;
      //gint64 upTime = arv_device_get_integer_feature_value( ad, "DeviceUptime" );      
      //BOOST_LOG_SEV(logger, notification) << "arv device DeviceUptime : " << upTime;
      
        frameCounter = 0;

        payload = arv_camera_get_payload (camera);
        BOOST_LOG_SEV(logger, notification) << "Camera payload : " << payload;

        #define FLIR_FRAME_RATE_REGISTER_OFFSET 0x083C
        #define FLIR_EXTENDED_SHUTTER_REGISTER_OFFSET 0x1028
        guint32 registerFrameRate, registerExtendedShutter;
	arv_device_read_register(ad, FLIR_FRAME_RATE_REGISTER_OFFSET, &registerFrameRate, NULL);
	arv_device_read_register(ad, FLIR_EXTENDED_SHUTTER_REGISTER_OFFSET, &registerExtendedShutter, NULL);

        BOOST_LOG_SEV(logger, notification) << "FLIR_FRAME_RATE_REGISTER_OFFSET       0x083C : " << std::hex << "0x" << registerFrameRate;
        BOOST_LOG_SEV(logger, notification) << "FLIR_EXTENDED_SHUTTER_REGISTER_OFFSET 0x1028 : " << std::hex << "0x" << registerExtendedShutter;
        
        pixFormat = arv_camera_get_pixel_format(camera);

        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax);
        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);
        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound min : " << exposureMin;
        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound max : " << exposureMax;
             
        BOOST_LOG_SEV(logger, notification) << "Camera gain bound min : " << gainMin;
        BOOST_LOG_SEV(logger, notification) << "Camera gain bound max : " << gainMax;

        ArvAuto gainAutoMode;
        gainAutoMode = arv_camera_get_gain_auto(camera);
        const char *gainAutoModeAtring;
        gainAutoModeAtring = arv_device_get_string_feature_value (ad, "GainAuto");
        BOOST_LOG_SEV(logger, notification) << "GainAuto : " << gainAutoModeAtring << " (" << gainAutoMode << ")";
        gainAutoMode = arv_camera_get_gain_auto( camera );
        BOOST_LOG_SEV(logger, notification) << "arv_camera_get_gain_auto : " << gainAutoMode;

        // added by MC
        // AcquisitionMode - FLIR sprcific
        bool ackFREnabled = arv_device_get_boolean_feature_value(ad, "AcquisitionFrameRateEnabled");
	BOOST_LOG_SEV(logger, notification) << "Camera Frame rate control enabled (AcquisitionFrameRateEnabled) : " << ackFREnabled;

        gint64 *values;
        guint n_values;
        const char ** valuesAsStrings;

        values = arv_device_get_available_enumeration_feature_values(ad, "AcquisitionMode", &n_values);
        valuesAsStrings = arv_device_get_available_enumeration_feature_values_as_strings(ad, "AcquisitionMode", &n_values);
	BOOST_LOG_SEV(logger, notification) << "AcquisitionModes :";
        for( int i=0; i< n_values; i++ )
          {
            BOOST_LOG_SEV(logger, notification) << "  [" << values[i] << ", " << valuesAsStrings[i] << "]";
          }

        ArvAcquisitionMode aaMode;
        aaMode = arv_camera_get_acquisition_mode( camera );
	BOOST_LOG_SEV(logger, notification) << "Camera ARV Acquisistion Mode : " << aaMode;

        // MC: ExposureAuto needs to be off - otherwise it is not possibleto set ExposureTime
        values = arv_device_get_available_enumeration_feature_values(ad, "ExposureAuto", &n_values);
        valuesAsStrings = arv_device_get_available_enumeration_feature_values_as_strings(ad, "ExposureAuto", &n_values);
	BOOST_LOG_SEV(logger, notification) << "ExposureAuto :";
        for( int i=0; i< n_values; i++ )
          {
            BOOST_LOG_SEV(logger, notification) << "  [" << values[i] << ", " << valuesAsStrings[i] << "]";
          }
        ArvAuto expAuto;
        expAuto = (ArvAuto)arv_device_get_integer_feature_value( ad, "ExposureAuto" );
	BOOST_LOG_SEV(logger, notification) << "Camera ExposureAuto Mode : " << expAuto;
        expAuto = arv_camera_get_exposure_time_auto( camera );
	BOOST_LOG_SEV(logger, notification) << "arv_camera_get_exposure_time_auto Mode : " << expAuto;        
        
        // MC: VideoMode 
        values = arv_device_get_available_enumeration_feature_values(ad, "VideoMode", &n_values);
        valuesAsStrings = arv_device_get_available_enumeration_feature_values_as_strings(ad, "VideoMode", &n_values);
	BOOST_LOG_SEV(logger, notification) << "VideoMode :";
        for( int i=0; i< n_values; i++ )
          {
            BOOST_LOG_SEV(logger, notification) << "  [" << values[i] << ", " << valuesAsStrings[i] << "]";
          }                
        gint64 videoMode;
        videoMode = arv_device_get_integer_feature_value( ad, "VideoMode" );
	BOOST_LOG_SEV(logger, notification) << "Original Camera VideoMode : " << videoMode;
        arv_device_set_integer_feature_value( ad, "VideoMode", 7 );
        videoMode = arv_device_get_integer_feature_value( ad, "VideoMode" );
	BOOST_LOG_SEV(logger, notification) << "Camera VideoMode : " << videoMode;        
        
	double frameRateMin, frameRateMax;
	// this ain't work arv_camera_get_frame_rate_bounds(camera, &frameRateMin, &frameRateMax);
	arv_device_get_float_feature_bounds (ad, "AcquisitionFrameRate", &frameRateMin, &frameRateMax);

	// arv_camera_set_frame_rate(camera, 32); // MC: here is hardcoded (was 30 FPS)
	//                                           but setFPS() is called later from AcqThread::prepareAcquisitionOnDevice()
        // MC TODO: remove this hard coded constant. Either use this->fps or value from config file 
	bool retVal = setFPS( 30 );
	
	BOOST_LOG_SEV(logger, notification) << "Camera frame rate bounds min : " << frameRateMin << "  max : " << frameRateMax;
   	
        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);
        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound min : " << exposureMin;
        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound max : " << exposureMax;

        capsString = arv_pixel_format_to_gst_caps_string(pixFormat);
        BOOST_LOG_SEV(logger, notification) << "Camera format : " << capsString;

        gain = arv_camera_get_gain(camera);
        BOOST_LOG_SEV(logger, notification) << "Camera gain : " << gain;

        exp = arv_camera_get_exposure_time(camera);
        BOOST_LOG_SEV(logger, notification) << "Camera exposure : " << exp;

        double expTimeAbs = arv_device_get_float_feature_value( ad, "ExposureTimeAbs" );
        BOOST_LOG_SEV(logger, notification) << "ExpTimeAbs      : " << expTimeAbs;
  
        cout << endl;

        cout << "DEVICE SELECTED : " << arv_camera_get_device_id(camera)    << endl;
        cout << "DEVICE NAME     : " << arv_camera_get_model_name(camera)   << endl;
        cout << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera)  << endl;
        cout << "PAYLOAD         : " << payload                             << endl;
        cout << "Start X         : " << mStartX                             << endl
             << "Start Y         : " << mStartY                             << endl;
        cout << "Width           : " << mWidth                               << endl
             << "Height          : " << mHeight                              << endl;
        cout << "Exp Range       : [" << exposureMin    << " - " << exposureMax   << "]"  << endl;
        cout << "Exp             : " << exp                                 << endl;
        cout << "Gain Range      : [" << gainMin        << " - " << gainMax       << "]"  << endl;
        cout << "Gain            : " << gain                                << endl;
        cout << "Fps             : " << fps                                 << endl;
        cout << "Type            : " << capsString                         << endl;

        cout << endl;

	
        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, NULL, NULL);

        if(stream == NULL){

            BOOST_LOG_SEV(logger, critical) << "Fail to create stream with arv_camera_create_stream()";
            return false;

        }

        // Push 50 buffer in the stream input buffer queue.
        for (int i = 0; i < 50; i++)
            arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

        return true;

    }

    void CameraUsbAravis::grabCleanse(){}

    bool CameraUsbAravis::acqStart(){

        BOOST_LOG_SEV(logger, notification) << "Set camera to CONTINUOUS MODE";
        arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS);

        BOOST_LOG_SEV(logger, notification) << "Set camera TriggerMode to Off";
        arv_device_set_string_feature_value(arv_camera_get_device (camera), "TriggerMode" , "Off");

        BOOST_LOG_SEV(logger, notification) << "Start acquisition on camera";
        arv_camera_start_acquisition(camera);

        return true;

    }

    void CameraUsbAravis::acqStop(){

        arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

        //cout << "Completed buffers = " << (unsigned long long) nbCompletedBuffers   << endl;
        //cout << "Failures          = " << (unsigned long long) nbFailures           << endl;
        //cout << "Underruns         = " << (unsigned long long) nbUnderruns          << endl;

        BOOST_LOG_SEV(logger, notification) << "Completed buffers = " << (unsigned long long) nbCompletedBuffers;
        BOOST_LOG_SEV(logger, notification) << "Failures          = " << (unsigned long long) nbFailures;
        BOOST_LOG_SEV(logger, notification) << "Underruns         = " << (unsigned long long) nbUnderruns;

        BOOST_LOG_SEV(logger, notification) << "Stopping acquisition...";
        arv_camera_stop_acquisition(camera);
        BOOST_LOG_SEV(logger, notification) << "Acquisition stopped.";

        BOOST_LOG_SEV(logger, notification) << "Unreferencing stream.";
        g_object_unref(stream);
        stream = NULL;

        BOOST_LOG_SEV(logger, notification) << "Unreferencing camera.";
        g_object_unref(camera);
        camera = NULL;

    }

    bool CameraUsbAravis::grabImage(Frame &newFrame){

        ArvBuffer *arv_buffer;
        //exp = arv_camera_get_exposure_time(camera);

        arv_buffer = arv_stream_timeout_pop_buffer(stream,12000000); //us; 12s
        char *buffer_data;
        size_t buffer_size;

        if(arv_buffer == NULL){

            throw runtime_error("arv_buffer is NULL");
            return false;

        }else{

            try{

                if(arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS){

                    BOOST_LOG_SEV(logger, normal) << "Success to grab a frame.";

                    buffer_data = (char *) arv_buffer_get_data (arv_buffer, &buffer_size);
		    
		    
                    //Timestamping.
                    string acquisitionDate = TimeDate::localDateTime(microsec_clock::universal_time(),"%Y:%m:%d:%H:%M:%S");
                    BOOST_LOG_SEV(logger, normal) << "Date : " << acquisitionDate;
                    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
                    acquisitionDate = to_iso_extended_string(time);
                    //BOOST_LOG_SEV(logger, normal) << "Date : " << acqDateInMicrosec;

                    Mat image;
                    CamPixFmt imgDepth = MONO8;
                    int saturateVal = 0;

                    if(pixFormat == ARV_PIXEL_FORMAT_MONO_8){

                        BOOST_LOG_SEV(logger, normal) << "Creating Mat 8 bits ...";

                        // MC: following Mat constructor does not copy data, only passes pounter to data (buffer_data)
			// it's  a fast constructor  ;-) 
                        image = Mat(mHeight, mWidth, CV_8UC1, buffer_data);
			if( buffer_size != (mHeight * mWidth * sizeof(unsigned char) ) ) 
			  {
			    std::cerr << "Creating Mat 8 bits ... "
				      << "WARNING: buffer_size=" << buffer_size << "  mHeight=" << mHeight << "  mWidth=" << mWidth << std::endl;
			  }
			if( buffer_size < (mHeight * mWidth * sizeof(unsigned char) ) )
			  {
			    std::cerr << "ERROR: CameraUsbAravis::grabImage(Frame &): arv_buffer_get_data returned smaller buffer size than mHeight*mWidth!" << std::endl;
			    exit(-1);
			  }
                        
                        imgDepth = MONO8;
                        saturateVal = 255;

                    }else if(pixFormat == ARV_PIXEL_FORMAT_MONO_12){

                        BOOST_LOG_SEV(logger, normal) << "Creating Mat 16 bits ...";
                        image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);
                        imgDepth = MONO12;
                        saturateVal = 4095;

                        //double t3 = (double)getTickCount();

                        if(shiftBitsImage){

                            BOOST_LOG_SEV(logger, normal) << "Shifting bits ...";


                                unsigned short * p;

                                for(int i = 0; i < image.rows; i++){
                                    p = image.ptr<unsigned short>(i);
                                    for(int j = 0; j < image.cols; j++)
                                        p[j] = p[j] >> 4;
                                }

                            BOOST_LOG_SEV(logger, normal) << "Bits shifted.";

                        }

                        //t3 = (((double)getTickCount() - t3)/getTickFrequency())*1000;
                        //cout << "> Time shift : " << t3 << endl;
                    } else if(pixFormat == ARV_PIXEL_FORMAT_MONO_16){

                        BOOST_LOG_SEV(logger, normal) << "Creating Mat 16 bits ...";
                        image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);
                        if( buffer_size != (mHeight * mWidth * sizeof(unsigned short) ) )
			  {
			    std::cerr << "Creating Mat 16 bits ... "
				      << "WARNING: buffer_size=" << buffer_size << "  mHeight=" << mHeight << "  mWidth=" << mWidth << std::endl;
			  }
			if( buffer_size < (mHeight * mWidth * sizeof(unsigned short) ) )
			  {
			    std::cerr << "ERROR: CameraUsbAravis::grabImage(Frame &): arv_buffer_get_data returned smaller buffer size than mHeight*mWidth!" << std::endl;
			    exit(-1);
			  }

                        imgDepth = MONO16;
                        saturateVal = 4095;
		    }

                    BOOST_LOG_SEV(logger, normal) << "Creating frame object ...";
                    newFrame = Frame(image, gain, exp, acquisitionDate);
                    BOOST_LOG_SEV(logger, normal) << "Setting date of frame ...";
                    //newFrame.setAcqDateMicro(acqDateInMicrosec);
                    BOOST_LOG_SEV(logger, normal) << "Setting fps of frame ...";
                    newFrame.mFps = fps;
                    newFrame.mFormat = imgDepth;
                    BOOST_LOG_SEV(logger, normal) << "Setting saturated value of frame ...";
                    newFrame.mSaturatedValue = saturateVal;
                    newFrame.mFrameNumber = frameCounter;
                    frameCounter++;

                    BOOST_LOG_SEV(logger, normal) << "Re-pushing arv buffer in stream ...";
                    arv_stream_push_buffer(stream, arv_buffer);

                    return true;

                }else{

                    switch(arv_buffer_get_status(arv_buffer)){

                        case 0 :
                            cout << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image"<<endl;
                            break;
                        case 1 :
                            cout << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared"<<endl;
                            break;
                        case 2 :
                            cout << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received"<<endl;
                            break;
                        case 3 :
                            cout << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets"<<endl;
                            break;
                        case 4 :
                            cout << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id"<<endl;
                            break;
                        case 5 :
                            cout << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space"<<endl;
                            break;
                        case 6 :
                            cout << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled"<<endl;
                            break;
                        case 7 :
                            cout << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion"<<endl;
                            break;

                    }

                    arv_stream_push_buffer(stream, arv_buffer);

                    return false;
                }

            }catch(exception& e){

                cout << e.what() << endl;
                BOOST_LOG_SEV(logger, critical) << e.what() ;
                return false;

            }
        }
    }


    bool CameraUsbAravis::grabSingleImage(Frame &frame, int camID){

        bool res = false;

        if(!createDevice(camID))
            return false;

        // MC: need to get ArvDevice to be get more complete access to camera features
        ArvDevice *ad;
        ad = arv_camera_get_device( camera );

        // MC: NO! setting frame rate seables frame rate! this->setFPS(1);
        
        // MC: disable frame rate to be able to set longer than 1/frame rate exposure time
        //     Now the actual frame rate does not control the exposure time limits and
        //     it is possible to set longer (extended) exposure time
        //        if (camera->priv->has_acquisition_frame_rate_enabled)
	arv_device_set_integer_feature_value( ad, "AcquisitionFrameRateEnabled", false );
	//		else
	//			arv_device_set_boolean_feature_value (camera->priv->device, "AcquisitionFrameRateEnable", 1);
        
        if(!setPixelFormat(frame.mFormat))
            return false;

        if(!setExposureTime(frame.mExposure))
            return false;
        // read back gain from camera HW so thet a real value gets written to FITS file
        exp = arv_camera_get_exposure_time(camera);
        frame.mExposure = exp;

        if(!setGain(frame.mGain))
            return false;
        // read back gain from camera HW so thet a real value gets written to FITS file
        gain = arv_camera_get_gain(camera);
        frame.mGain = gain;

        if(frame.mWidth > 0 && frame.mHeight > 0) {

            setSize(frame.mStartX, frame.mStartY, frame.mWidth, frame.mHeight, 1);

        }else{

            int sensor_width, sensor_height;

            arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height);

            // Use maximum sensor size.
            arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height);
            arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight);

        }

        payload = arv_camera_get_payload (camera);

        pixFormat = arv_camera_get_pixel_format (camera);

        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);

        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax);

        capsString = arv_pixel_format_to_gst_caps_string(pixFormat);

        double expTimeAbs = arv_device_get_float_feature_value( ad, "ExposureTimeAbs" );

	double frameRateMin, frameRateMax;
	// this ain't work arv_camera_get_frame_rate_bounds(camera, &frameRateMin, &frameRateMax);
	arv_device_get_float_feature_bounds (ad, "AcquisitionFrameRate", &frameRateMin, &frameRateMax);

        fps = arv_camera_get_frame_rate(camera);
        
	BOOST_LOG_SEV(logger, notification) << "DEVICE SELECTED : " << arv_camera_get_device_id(camera);
        BOOST_LOG_SEV(logger, notification) << "DEVICE NAME     : " << arv_camera_get_model_name(camera);
        BOOST_LOG_SEV(logger, notification) << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera);
        BOOST_LOG_SEV(logger, notification) << "PAYLOAD         : " << payload;
        BOOST_LOG_SEV(logger, notification) << "Width           : " << mWidth;
        BOOST_LOG_SEV(logger, notification) << "Height          : " << mHeight;
        BOOST_LOG_SEV(logger, notification) << "Exp Range       : [" << exposureMin << " - " << exposureMax << "]";
        BOOST_LOG_SEV(logger, notification) << "Exp             : " << exp;
        BOOST_LOG_SEV(logger, notification) << "ExpTimeAbs      : " << expTimeAbs;
        BOOST_LOG_SEV(logger, notification) << "Gain Range      : [" << gainMin << " - " << gainMax << "]";
        BOOST_LOG_SEV(logger, notification) << "Gain            : " << gain;
	BOOST_LOG_SEV(logger, notification) << "Fps Range       : [" << frameRateMin << " - " << frameRateMax << "]";
        BOOST_LOG_SEV(logger, notification) << "Fps             : " << fps;
        BOOST_LOG_SEV(logger, notification) << "Type            : " << capsString;

        bool ackFREnabled = arv_device_get_boolean_feature_value(ad, "AcquisitionFrameRateEnabled");
	BOOST_LOG_SEV(logger, notification) << "Camera Frame rate control enabled (AcquisitionFrameRateEnabled) : " << ackFREnabled;

        gint64 *values;
        guint n_values;
        values = arv_device_get_available_enumeration_feature_values(ad, "AcquisitionMode", &n_values);
        const char ** valuesAsStrings;
        valuesAsStrings = arv_device_get_available_enumeration_feature_values_as_strings(ad, "AcquisitionMode", &n_values);
	BOOST_LOG_SEV(logger, notification) << "AcquisitionModes :";
        for( int i=0; i< n_values; i++ )
          {
            BOOST_LOG_SEV(logger, notification) << "  [" << values[i] << ", " << valuesAsStrings[i] << "]";
          }
        
        ArvAcquisitionMode aaMode;
        aaMode = arv_camera_get_acquisition_mode( camera );
	BOOST_LOG_SEV(logger, notification) << "Camera ARV Acquisistion Mode : " << aaMode;
        
        cout << endl;

        cout << "DEVICE SELECTED : " << arv_camera_get_device_id(camera)    << endl;
        cout << "DEVICE NAME     : " << arv_camera_get_model_name(camera)   << endl;
        cout << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera)  << endl;
        cout << "PAYLOAD         : " << payload                             << endl;
        cout << "Width           : " << mWidth                               << endl
             << "Height          : " << mHeight                              << endl;
        cout << "Exp Range       : [" << exposureMin    << " - " << exposureMax   << "]"  << endl;
        cout << "Exp             : " << exp                                 << endl;
        cout << "Gain Range      : [" << gainMin        << " - " << gainMax       << "]"  << endl;
        cout << "Gain            : " << gain                                << endl;
        cout << "Fps             : " << fps                                 << endl;
        cout << "Type            : " << capsString                         << endl;

        cout << endl;
        
        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, NULL, NULL);

        if(stream != NULL){

            // Push 50 buffer in the stream input buffer queue.
            arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

            // Set acquisition mode to single frame.
            arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_SINGLE_FRAME);
            aaMode = arv_camera_get_acquisition_mode( camera );
            BOOST_LOG_SEV(logger, notification) << "Camera ARV Acquisistion Mode : " << aaMode;

            // Very usefull to avoid arv buffer timeout status
            sleep(2);
                        
            //Timestamping
            boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
            BOOST_LOG_SEV(logger, notification) << "Timestamp before arv_camera_start_acquisition call : " << to_iso_extended_string(time);
            
            // Start acquisition.
            arv_camera_start_acquisition(camera);

            cout << ">> Acquisition in progress... (Please wait)" << endl;

            // Get image buffer.
            ArvBuffer *arv_buffer = arv_stream_timeout_pop_buffer(stream, 2 * frame.mExposure + 5000000); //us
            
            boost::posix_time::ptime time2 = boost::posix_time::microsec_clock::universal_time();
            BOOST_LOG_SEV(logger, notification) << "Timestamp after arv_stream_timeout_pop_buffer      : " << to_iso_extended_string(time2);
            time = time2 - boost::posix_time::microseconds( exp );
            BOOST_LOG_SEV(logger, notification) << "Timestamp after captured - exposure time           : " << to_iso_extended_string(time);

            char *buffer_data;
            size_t buffer_size;

            if (arv_buffer != NULL){

                if(arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS){

                    buffer_data = (char *) arv_buffer_get_data (arv_buffer, &buffer_size);

                    if(pixFormat == ARV_PIXEL_FORMAT_MONO_8){

                        Mat image = Mat(mHeight, mWidth, CV_8UC1, buffer_data);
                        image.copyTo(frame.mImg);

                    }else if(pixFormat == ARV_PIXEL_FORMAT_MONO_12){

                        // Unsigned short image.
                        Mat image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);

                        // http://www.theimagingsource.com/en_US/support/documentation/icimagingcontrol-class/PixelformatY16.htm
                        // Some sensors only support 10-bit or 12-bit pixel data. In this case, the least significant bits are don't-care values.
                        if(shiftBitsImage){
                            unsigned short * p;
                            for(int i = 0; i < image.rows; i++){
                                p = image.ptr<unsigned short>(i);
                                for(int j = 0; j < image.cols; j++) p[j] = p[j] >> 4;
                            }
                        }

                        image.copyTo(frame.mImg);

                    }

                    frame.mDate = TimeDate::splitIsoExtendedDate(to_iso_extended_string(time));
                    frame.mFps = arv_camera_get_frame_rate(camera);

                    res = true;

                }else{

                    switch(arv_buffer_get_status(arv_buffer)){

                        case 0 :

                            cout << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image"<<endl;

                            break;

                        case 1 :

                            cout << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared"<<endl;

                            break;

                        case 2 :

                            cout << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received"<<endl;

                            break;

                        case 3 :

                            cout << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets"<<endl;

                            break;

                        case 4 :

                            cout << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id"<<endl;

                            break;

                        case 5 :

                            cout << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space"<<endl;

                            break;

                        case 6 :

                            cout << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled"<<endl;

                            break;

                        case 7 :

                            cout << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion"<<endl;

                            break;


                    }

                    res = false;

                }

                arv_stream_push_buffer(stream, arv_buffer);

           }else{

                BOOST_LOG_SEV(logger, fail) << "Fail to pop buffer from stream.";
                res = false;

           }

            arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

            cout << ">> Completed buffers = " << (unsigned long long) nbCompletedBuffers    << endl;
            cout << ">> Failures          = " << (unsigned long long) nbFailures           << endl;
            //cout << ">> Underruns         = " << (unsigned long long) nbUnderruns          << endl;

            // Stop acquisition.
            arv_camera_stop_acquisition(camera);

            g_object_unref(stream);
            stream = NULL;
            g_object_unref(camera);
            camera = NULL;

        }

        return res;

    }

    void CameraUsbAravis::saveGenicamXml(string p){

        const char *xml;

        size_t size;

        xml = arv_device_get_genicam_xml (arv_camera_get_device(camera), &size);

        if (xml != NULL){

            ofstream infFile;
            string infFilePath = p + "genicam.xml";
            infFile.open(infFilePath.c_str());
            infFile << string ( xml, size );
            infFile.close();

        }

    }

    //https://github.com/GNOME/aravis/blob/b808d34691a18e51eee72d8cac6cfa522a945433/src/arvtool.c
    void CameraUsbAravis::getAvailablePixelFormats() {

        ArvGc *genicam;
        ArvDevice *device;
        ArvGcNode *node;

        if(camera != NULL) {

            device = arv_camera_get_device(camera);
            genicam = arv_device_get_genicam(device);
            node = arv_gc_get_node(genicam, "PixelFormat");

            if (ARV_IS_GC_ENUMERATION (node)) {

                const GSList *childs;
                const GSList *iter;
                vector<string> pixfmt;

                cout << ">> Device pixel formats :" << endl;

                childs = arv_gc_enumeration_get_entries (ARV_GC_ENUMERATION (node));
                for (iter = childs; iter != NULL; iter = iter->next) {
                    if (arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (iter->data), NULL)) {

                        if(arv_gc_feature_node_is_available (ARV_GC_FEATURE_NODE (iter->data), NULL)) {

                            {
                                string fmt = string(arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE (iter->data)));
                                std::transform(fmt.begin(), fmt.end(),fmt.begin(), ::toupper);
                                pixfmt.push_back(fmt);
                                cout << "- " << fmt << endl;

                            }
                        }
                    }
                }

                // Compare found pixel formats to currently formats supported by freeture

                cout << endl <<  ">> Available pixel formats :" << endl;
                EParser<CamPixFmt> fmt;

                for( int i = 0; i != pixfmt.size(); i++ ) {

                    if(fmt.isEnumValue(pixfmt.at(i))) {

                        cout << "- " << pixfmt.at(i) << " available --> ID : " << fmt.parseEnum(pixfmt.at(i)) << endl;

                    }

                }

            }else {

                cout << ">> Available pixel formats not found." << endl;

            }

            g_object_unref(device);

        }

    }

    void CameraUsbAravis::getExposureBounds(double &eMin, double &eMax){

        double exposureMin = 0.0;
        double exposureMax = 0.0;

        arv_camera_get_exposure_time_bounds(camera, &exposureMin, &exposureMax);

        eMin = exposureMin;
        eMax = exposureMax;

    }

    double CameraUsbAravis::getExposureTime(){

        return arv_camera_get_exposure_time(camera);

    }

    void CameraUsbAravis::getGainBounds(int &gMin, int &gMax){

        double gainMin = 0.0;
        double gainMax = 0.0;

        arv_camera_get_gain_bounds(camera, &gainMin, &gainMax);

        gMin = gainMin;
        gMax = gainMax;

    }

    bool CameraUsbAravis::getPixelFormat(CamPixFmt &format){

        ArvPixelFormat pixFormat = arv_camera_get_pixel_format(camera);

        switch(pixFormat){

            case ARV_PIXEL_FORMAT_MONO_8 :

                format = MONO8;

                break;

            case ARV_PIXEL_FORMAT_MONO_12 :

                format = MONO12;

                break;

	    case ARV_PIXEL_FORMAT_MONO_16 :
	        format = MONO16;
	        break;

            default :

                return false;

                break;

        }

        return true;
    }


    bool CameraUsbAravis::getFrameSize(int &w, int &h) {

        if(camera != NULL) {

            int ww = 0, hh = 0;
            arv_camera_get_region(camera, NULL, NULL, &ww, &h);
            w = ww;
            h = hh;

        }

        return false;

    }

    bool CameraUsbAravis::getFPS(double &value){

        if(camera != NULL) {

            value = arv_camera_get_frame_rate(camera);
            return true;

        }

        return false;

    }

    string CameraUsbAravis::getModelName(){

        return arv_camera_get_model_name(camera);

    }

    bool CameraUsbAravis::setExposureTime(double val){
      
      double expMin, expMax;
      
      arv_camera_get_exposure_time_bounds(camera, &expMin, &expMax);
      
      ArvDevice *ad;
      ad = arv_camera_get_device( camera );
      
      if(camera != NULL)
        {
          if(val < expMin && val > expMax)
            {
              
              BOOST_LOG_SEV(logger, warning) << "Exposure value (" << val << ") is not in range [ " << expMin << " - " << expMax << " ]";
              cout << "Exposure value (" << val << ") is not in range [ " << expMin << " - " << expMax << " ]" << endl;
            }
          else
            {
            
            exp = arv_camera_get_exposure_time(camera);
            BOOST_LOG_SEV(logger, notification) << "Exp read pre set  : " << exp;
            
            // MC: disable auto exposure control if set
            gint64 expAuto;
            expAuto = arv_device_get_integer_feature_value( ad, "ExposureAuto" );
            // ExposureAuto :
            //   [0, Off]
            //   [1, Once]
            //   [2, Continuous]
            if( expAuto != 0 )
              {
                BOOST_LOG_SEV(logger, notification) << "Original Camera ExposureAuto Mode : " << expAuto;
                arv_device_set_integer_feature_value( ad, "ExposureAuto", 0 );
                expAuto = arv_device_get_integer_feature_value( ad, "ExposureAuto" );
                BOOST_LOG_SEV(logger, notification) << "Camera ExposureAuto Mode : " << expAuto;
              }
            
            arv_camera_set_exposure_time(camera, val);
            // MC: no check for bounds in arv_camera_set_exposure_time(), but writes as follows: 
            /* arv_device_set_float_feature_value (camera->priv->device,
							    camera->priv->has_exposure_time ?
							    "ExposureTime" :
							    "ExposureTimeAbs", exposure_time_us);
            */
            exp = arv_camera_get_exposure_time(camera);
            BOOST_LOG_SEV(logger, notification) << "Exp read after set: " << exp;
            // check if exposure time is pretty much what was requested
            // (ie does not differ more than 1/100 of average from requested and read back value)
            if( abs( (val-exp)/((val+exp)/(double)2) ) > ((double)1/(double)100) )
              {
                BOOST_LOG_SEV(logger, fail) << "exposure time read back (" << exp << ") differs from value requested to be set (" << val << ")";
              }
            else
              {
                BOOST_LOG_SEV(logger, notification) << "exposure time read back (" << exp << ") is pretty much same as value requested to be set (" << val << ")";
              }
            // MC: need to get ArvDevice to be get more complete access to camera features
            ArvDevice *ad;
            ad = arv_camera_get_device( camera );
            double expTimeAbs = arv_device_get_float_feature_value( ad, "ExposureTimeAbs" );
            BOOST_LOG_SEV(logger, notification) << "ExpTimeAbs      : " << expTimeAbs;
            
          }
          
            return true;

        }

        return false;
    }

    bool CameraUsbAravis::setGain(int val){

        double gMin, gMax;

        arv_camera_get_gain_bounds (camera, &gMin, &gMax);

        ArvDevice *ad;
        ad = arv_camera_get_device( camera );

        if (camera != NULL){

            if((double)val >= gMin && (double)val <= gMax){

            // MC: disable auto exposure control if set
            gint64 gainAuto;
            gainAuto = arv_device_get_integer_feature_value( ad, "GainAuto" );
            // GainAuto :
            //   [0, Off]
            //   [1, Once]
            //   [2, Continuous]
            if( gainAuto != 0 )
              {
                BOOST_LOG_SEV(logger, notification) << "Original Camera GainAuto Mode : " << gainAuto;
                arv_device_set_integer_feature_value( ad, "GainAuto", 0 );
                gainAuto = arv_device_get_integer_feature_value( ad, "GainAuto" );
                BOOST_LOG_SEV(logger, notification) << "Camera GainAuto Mode : " << gainAuto;
              }
            
            arv_camera_set_gain (camera, (double)val);

            // read back to check if value accepted by camera
            gain = arv_camera_get_gain (camera);
            BOOST_LOG_SEV(logger, notification) << "Gain read after set: " << gain;
            // check if gain is pretty much what was requested
            // (ie does not differ more than 1/100 of average from requested and read back value)
            if( abs( (val-gain)/((val+gain)/(double)2) ) > ((double)1/(double)100) )
              {
                BOOST_LOG_SEV(logger, fail) << "gain read back (" << gain << ") differs from value requested to be set (" << val << ")";
              }
            else
              {
                BOOST_LOG_SEV(logger, notification) << "gain time read back (" << gain << ") is pretty much same as value requested to be set (" << val << ")";
              }
              
            }else{

                cout << "CameraUsbAravis::setGain(int): Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]" << endl;
                BOOST_LOG_SEV(logger, fail) << "CameraUsbAravis::setGain(int): Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]";
                return false;

            }

        return true;

        }

        return false;

    }

    bool CameraUsbAravis::setFPS(double val){

        if (camera != NULL){
          // MC: need to get ArvDevice to be get more complete access to camera features
          ArvDevice *ad;
          ad = arv_camera_get_device( camera );
          
          double frameRateMin, frameRateMax;
          // this ain't work arv_camera_get_frame_rate_bounds(camera, &frameRateMin, &frameRateMax);
          arv_device_get_float_feature_bounds (ad, "AcquisitionFrameRate", &frameRateMin, &frameRateMax);
          
          if(val >= frameRateMin && val <= frameRateMax)
            // MC note: arv_camera_set_frame_rate() silently limits written value to bounds
            //  also it does following for FLIR cameras
            /*		case ARV_CAMERA_VENDOR_POINT_GREY_FLIR:
			arv_device_set_string_feature_value (camera->priv->device, "TriggerSelector", "FrameStart");
			arv_device_set_string_feature_value (camera->priv->device, "TriggerMode", "Off");
			if (camera->priv->has_acquisition_frame_rate_enabled)
				arv_device_set_integer_feature_value (camera->priv->device, "AcquisitionFrameRateEnabled", 1);
			else
				arv_device_set_boolean_feature_value (camera->priv->device, "AcquisitionFrameRateEnable", 1);
			arv_device_set_string_feature_value (camera->priv->device, "AcquisitionFrameRateAuto", "Off");
			arv_device_set_float_feature_value (camera->priv->device, "AcquisitionFrameRate", frame_rate);
            */
            {
              arv_camera_set_frame_rate(camera, val);
              // read back what was set
              this->fps = arv_camera_get_frame_rate(camera);
              BOOST_LOG_SEV(logger, notification) << "Camera frame rate : " << this->fps;
              // check if frame rate is pretty much what was requested
              // (ie does not differ more than 1/100 of average from requested and read back value)
              if( abs( (val-fps)/((val+fps)/(double)2) ) > ((double)1/(double)100) )
                {
                  BOOST_LOG_SEV(logger, fail) << "frame rate read back (" << fps << ") differs from value requested (" << val << ")";
                }
              else
                {
                  BOOST_LOG_SEV(logger, notification) << "frame rate read back (" << fps << ") is pretty much same as value requested (" << val << ")";
                }
            }
          else
            {
              cout << "CameraUsbAravis::setFPS(double): FPS value (" << val << ") is not in range [ " << frameRateMin << " - " << frameRateMax << " ]" << endl;
              BOOST_LOG_SEV(logger, fail) << "CameraUsbAravis::setFPS(double): FPS value (" << val << ") is not in range [ " << frameRateMin << " - " << frameRateMax << " ]";
              return false;
              
            }
          return true;

        }
        return false;

    }

    bool CameraUsbAravis::setPixelFormat(CamPixFmt depth){

        if (camera != NULL){

            switch(depth){

                case MONO8 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8);
                    }
                    break;

                case MONO12 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_12);
                    }
                    break;

            }

            return true;
        }

        return false;

    }

 bool CameraUsbAravis::setFrameSize(int startx, int starty, int width, int height, bool customSize) {

        if (camera != NULL){
            if(customSize) {

                if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                    cout << "Starting from : " << mStartX << "," << mStartY;
                    BOOST_LOG_SEV(logger, notification) << "Starting from : " << mStartX << "," << mStartY;
                } else {
                    BOOST_LOG_SEV(logger, warning) << "OffsetX, OffsetY are not available: cannot set offset.";
                }
                arv_camera_set_region(camera, startx, starty, width, height);
                arv_camera_get_region (camera, &mStartX, &mStartY, &mWidth, &mHeight);

            // Default is maximum size
            } else {

                int sensor_width, sensor_height;

                arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height);
                BOOST_LOG_SEV(logger, notification) << "Camera sensor size : " << sensor_width << "x" << sensor_height;

                arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height);
                arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight);

            }
            return true;
        }
        return false;
    }

// Reset of BFLY-U3-23S6M-C with FW V1.9.3
// the LED turns orange and stays like that(sleep mode)
// camera needs to be re-connecetd  getCamerasList(); createDevice(int id);
void CameraUsbAravis::deviceReset() {
        // MC: need to get ArvDevice to be get more complete access to camera features
        ArvDevice *ad;
        ad = arv_camera_get_device( camera );
        BOOST_LOG_SEV(logger, notification) << "Initiating camera reset... ";
        arv_device_execute_command(ad, "DeviceReset");
        sleep( 10 );
        BOOST_LOG_SEV(logger, notification) << "Camera reset done, with 10s wait afterwards.";
}

#endif
