/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Evidential FastSLAM nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cstdlib>
#include <clocale>

#include "efslam/io/carmen/CarmenReader.h"
#include "efslam/utils/Convenience.h"
#include "efslam/utils/Log.h"

namespace efs {

constexpr	char CarmenReader::LASER_OLD_STR[];
constexpr	char CarmenReader::LASER_NEW_STR[];


CarmenReader::CarmenReader( std::istream &inputStream ) :
		m_in( inputStream )
{
}

CarmenReader::~CarmenReader() {
}


bool
CarmenReader::nextReading( PoseSE2 *pose, std::vector<double> *scan, std::string *timestamp ) {
	bool found = false;

	scan->clear();
	while( m_in.good() && !found ) {
		std::string line;
		std::getline( m_in, line );

		int numReadingIdx	= LASER_OLD_NUM_READINGS,
			readingsIdx		= LASER_OLD_READINGS,
			poseXIdx		= LASER_OLD_POSE_X,
			poseYIdx		= LASER_OLD_POSE_Y,
			posePhiIdx		= LASER_OLD_POSE_PHI,
			timestampIdx	= LASER_OLD_TIMESTAMP,
			minValues		= LASER_OLD_MIN_VALUES;

		LineTypeE	lineType = TYPE_UNKNOWN;

		if( line.compare( 0, strlen( LASER_OLD_STR ), LASER_OLD_STR ) == 0 ) {
			lineType = LASER_OLD;
		} else if( line.compare( 0, strlen( LASER_NEW_STR ), LASER_NEW_STR ) == 0 ) {
			lineType = LASER_NEW;

			numReadingIdx	= LASER_NEW_NUM_READINGS,
			readingsIdx		= LASER_NEW_READINGS,
			poseXIdx		= LASER_NEW_POSE_X,
			poseYIdx		= LASER_NEW_POSE_Y,
			posePhiIdx		= LASER_NEW_POSE_PHI,
			timestampIdx	= LASER_NEW_TIMESTAMP,
			minValues		= LASER_NEW_MIN_VALUES;
		}


		if( lineType != TYPE_UNKNOWN ) {
			found = true;

			auto splitted = split( line, ' ' );

			int numReadings		= std::stoi( splitted[numReadingIdx] ),
				numRemissions	= 0;

			if( lineType == LASER_NEW ) {
				numRemissions 	= std::stoi( splitted[numReadings + LASER_NEW_NUM_REMISSIONS] );
			}

			int offset = numReadings + numRemissions;

			if( (int) splitted.size() < offset + minValues ) {
				l_wrn( "Wrong number of fields: Expected at least " << (offset + minValues) << " got " << splitted.size() );
				found = false;
				continue;
			}

			scan->reserve( numReadings );
			for( int i = 0; i < numReadings; i++ )
				scan->push_back( std::stod( splitted[readingsIdx + i] ) );

			/*
			for( int i = numReadings; i < splitted.size(); i++ )
				std::cout << splitted[i] << std::endl;
			*/

			pose->x() 	= std::stof( splitted[poseXIdx + offset] );
			pose->y() 	= std::stof( splitted[poseYIdx + offset] );
			pose->phi()	= std::stof( splitted[posePhiIdx + offset] );
			//std::cout << pose << std::endl;

			if( timestamp )
				*timestamp = splitted[timestampIdx + offset];
			//std::cout << splitted[timestampIdx + offset] << std::endl;
		}

	}

	return found;
}


} /* namespace efs */
