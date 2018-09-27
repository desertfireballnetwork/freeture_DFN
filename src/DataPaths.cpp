/*
                                DataPaths.cpp

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
* \file    DataPaths.cpp
* \author  Hadrien Devillepoix -- Desert Fireball Network
* \version 1.0
* \date    27/09/2018
* \brief   Manage data paths.
*/

#include "DataPaths.h"



string DataPaths::getSessionPath(string data_path, Date date) {
    
    char hostname [1024] = "";
    const int result = gethostname(hostname, sizeof hostname);
        
    string cam_hostname = str(hostname);
    
    string fp = data_path + "/" + cam_hostname + "/" + TimeDate::getYYYY(date)  + "/" + TimeDate::getMM(date)  + "/" + TimeDate::getYYYY-MM-DD(date) + "_"  cam_hostname + "_" + "allskyvideo" + "/";
    
    // /data0/DFNEXT009/2018/09/2018-09-25_DFNEXT009_allskyvideo/
    
    return fp;
    
    
}


