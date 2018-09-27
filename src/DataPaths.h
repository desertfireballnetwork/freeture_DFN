/*
                                DataPaths.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2018    Hadrien Devillepoix
*                               Desert Fireball Network
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
*   Last modified:      27/09/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


/**
* \file    DataPaths.h
* \author  Hadrien Devillepoix -- Desert Fireball Network
* \version 1.0
* \date    27/09/2018
* \brief   Manage data paths.
*/

#pragma once

#include <string>
#include <iostream>
#include <unistd.h>
#include <list>

#include "SParam.h"

using namespace std;
using namespace boost::posix_time;

class DataPaths {

    public :


        /**
        * Get session path
        *
        * @param str base path where data is sotred
        * @param date Vector of date : YYYY, MM, DD, hh, mm, ss.
        * @return path string
        */
        static string getSessionPath(string data_path, Date date);


};

