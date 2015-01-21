/*
				Fits2D.h

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
*	Last modified:		28/11/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @file    Fits2D.h
 * @author  Yoan Audureau -- FRIPON-GEOPS-UPSUD
 * @version 1.0
 * @date    28/11/2014
 */

#pragma once

#include "includes.h"

#ifdef CFITSIO_H
  #include CFITSIO_H
#else
  #include "fitsio.h"
#endif

#include "TimeDate.h"
#include "ELogSeverityLevel.h"
#include "EImgBitDepth.h"
#include "Fits.h"
#include "Conversion.h"

using namespace std;
using namespace boost::posix_time;

namespace logging	= boost::log;
namespace sinks		= boost::log::sinks;
namespace attrs		= boost::log::attributes;
namespace src		= boost::log::sources;
namespace expr		= boost::log::expressions;
namespace keywords	= boost::log::keywords;

class Fits2D : public Fits{

    //Attributes.

	private :

		src::severity_logger< LogSeverityLevel > log;

		string fitsPath;

		Fits fits;

    //Methods.

	public:

                Fits2D          (string recPath, const Fits & f):
                                fitsPath(recPath), fits(f){};

                Fits2D          ();

                ~Fits2D         (void);

		bool    writeFits       (Mat img,
                                 ImgBitDepth imgType,
                                 int nb,
                                 bool filenameWithDate,
                                 string fileName);

		bool    readFits32F     (Mat &img, string filePath);
		bool    readFits16US    (Mat &img, string filePath);
		bool    readFits16S     (Mat &img, string filePath);
		bool    readFits8UC     (Mat &img, string filePath);
		bool    readFits8C      (Mat &img, string filePath);

		bool    readIntKeyword  (string filePath, string keyword, int &value);

    private:

        bool    printerror      (int status, string errorMsg);
		bool    printerror      (string errorMsg);
		void    printerror      (int status);
		bool    writeKeywords   (fitsfile *fptr);

};


