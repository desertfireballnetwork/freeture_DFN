/*
                                Fits.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                               GEOPS-UPSUD-CNRS
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
*   Last modified:      21/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Fits.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    04/12/2014
*/

#pragma once

#include "SParam.h"
#include "config.h"

#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace std;

class Fits {

     public :

        string  STATION;

        // Fits Header Keywords Name.

        string  kFILENAME;
        string  kDATE;
        string  kDATEOBS;
        string  kOBSMODE;
        double  kELAPTIME;
        double  kEXPOSURE;
        double  kONTIME;
        string  kFILTER;
        string  kTELESCOP;
        string  kOBSERVER;
        string  kINSTRUME;
        string  kCAMERA;
        double  kFOCAL;
        double  kAPERTURE;
        double  kSITELONG;
        double  kSITELAT;
        double  kSITEELEV;
        string  kGPS_LOCK;
        double  kXPIXEL;
        double  kYPIXEL;
        int     kGAINDB;
        double  kSATURATE;
        string  kPROGRAM;
        string  kCREATOR;
        double  kBZERO;
        double  kBSCALE;
        string  kRADESYS;
        string  kTIMESYS;
        double  kEQUINOX;
        string  kCTYPE1;
        string  kCTYPE2;
        string  kCTYPE3;
        string  kTIMEUNIT;
        double  kCD1_1;
        double  kCD1_2;
        double  kCD2_1;
        double  kCD2_2;
        double  kCD3_3;
        double  kCD1_3;
        double  kCD2_3;
        double  kCD3_1;
        double  kCD3_2;
        int     kCRPIX1;
        int     kCRPIX2;
        int     kCRPIX3;
        double  kCRVAL1;
        double  kCRVAL2;
        double  kK1;
        double  kK2;
        string  kCOMMENT;

        // Fits Header Keywords Comments.

        string  cFILENAME;
        string  cDATE;
        string  cDATEOBS;
        string  cOBSMODE;
        string  cELAPTIME;
        string  cEXPOSURE;
        string  cONTIME;
        string  cFILTER;
        string  cTELESCOP;
        string  cOBSERVER;
        string  cINSTRUME;
        string  cCAMERA;
        string  cFOCAL;
        string  cAPERTURE;
        string  cSITELONG;
        string  cSITELAT;
        string  cSITEELEV;
        string  cGPS_LOCK;
        string  cXPIXEL;
        string  cYPIXEL;
        string  cGAINDB;
        string  cSATURATE;
        string  cPROGRAM;
        string  cCREATOR;
        string  cBZERO;
        string  cBSCALE;
        string  cRADESYS;
        string  cTIMESYS;
        string  cEQUINOX;
        string  cCTYPE1;
        string  cCTYPE2;
        string  cCTYPE3;
        string  cTIMEUNIT;
        string  cCD1_1;
        string  cCD1_2;
        string  cCD2_1;
        string  cCD2_2;
        string  cCD3_3;
        string  cCD1_3;
        string  cCD2_3;
        string  cCD3_1;
        string  cCD3_2;
        string  cCRPIX1;
        string  cCRPIX2;
        string  cCRPIX3;
        string  cCRVAL1;
        string  cCRVAL2;
        string  cK1;
        string  cK2;
        string  cCOMMENT;

    public :

        Fits();

        ~Fits();

        void loadKeys(fitskeysParam fkp, stationParam sp);
};

