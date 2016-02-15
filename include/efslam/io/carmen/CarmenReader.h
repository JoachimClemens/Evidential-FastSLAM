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

#ifndef EFS_CARMENREADER_H_
#define EFS_CARMENREADER_H_

#include <vector>

#include "efslam/utils/PoseSE2.h"

namespace efs {

class CarmenReader {
public:
			CarmenReader( std::istream &inputStream );
	virtual ~CarmenReader();

	bool	nextReading( PoseSE2 *pose, std::vector<double> *scan, std::string *timestamp = nullptr );

private:
	static constexpr	char LASER_OLD_STR[] = "FLASER";
	static constexpr	char LASER_NEW_STR[] = "ROBOTLASER";

	enum LineTypeE {
		TYPE_UNKNOWN = -1,
		LASER_OLD,
		LASER_NEW
	};

	enum {
		LASER_OLD_NUM_READINGS	= 1,
		LASER_OLD_READINGS		= 2, // if numReadings == 0, than LASER_OLD_POSE_X is at this position
		LASER_OLD_POSE_X 		= 2, // + numReadings
		LASER_OLD_POSE_Y 		= 3, // + numReadings
		LASER_OLD_POSE_PHI 		= 4, // + numReadings
		LASER_OLD_TIMESTAMP		= 8, // + numReadings
		LASER_OLD_MIN_VALUES		 // + numReadings
	};

	enum {
		// LASER_NEW_TYPE
		LASER_NEW_START_ANGLE		= 2,
		LASER_NEW_FOV				= 3,
		LASER_NEW_ANGULAR_RES		= 4,
		LASER_NEW_MAX_RANGE			= 5,
		LASER_NEW_ACCURANCY			= 6,
		// LASER_NEW_REMISSION_MODE
		LASER_NEW_NUM_READINGS		= 8,
		LASER_NEW_READINGS			= 9, // if numReadings == 0, than LASER_NEW_NUM_REMISSIONS is at this position
		LASER_NEW_NUM_REMISSIONS	= 9, // + numReadings
		// LASER_NEW_LASER_X
		// LASER_NEW_LASER_Y
		// LASER_NEW_LASER_PHI
		LASER_NEW_POSE_X 			= 13, // + numReadings + numRemissions
		LASER_NEW_POSE_Y 			= 14, // + numReadings + numRemissions
		LASER_NEW_POSE_PHI 			= 15, // + numReadings + numRemissions
		LASER_NEW_TIMESTAMP			= 21, // + numReadings + numRemissions
		LASER_NEW_MIN_VALUES
	};
	std::istream	&m_in;
};

} /* namespace efs */

#endif /* EFS_CARMENREADER_H_ */
