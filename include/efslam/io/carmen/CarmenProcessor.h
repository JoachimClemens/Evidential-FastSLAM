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

#ifndef EFS_CARMENPROCESSOR_H_
#define EFS_CARMENPROCESSOR_H_

#include <list>

#include "CarmenReader.h"
#include "CarmenGtReader.h"

namespace efs {

template<typename SLAM>
class CarmenProcessor {
public:
	using PointType		= typename SLAM::PointType;
	using Scan 			= typename SLAM::ScanType;
	using Trajectory	= typename SLAM::Trajectory;

					CarmenProcessor( SLAM *slam, std::istream &inputStream, std::istream *gtInputStream = nullptr );
					~CarmenProcessor();

	inline uint64_t processAll();
	inline bool		processNext( bool *scanProcessed = nullptr );

protected:
	SLAM			*m_slam;
	CarmenReader	m_reader;
	CarmenGtReader	*m_gtReader;

	double			m_laserStartAngle,
					m_laserAngularRes;
	bool			m_first;
	PoseSE2			m_lastPose,
					m_lastGtPose,
					m_lastEstPose;

	std::list<std::string>	m_timestamps;
	std::list<PoseSE2>		m_gtTrajectory;
	std::list<double>		m_poseError;

public:
	GETTER( timestamps );
	GETTER( gtTrajectory );
	GETTER( poseError );

private:
	CarmenProcessor( const CarmenProcessor &other ) {}; // shall not be used
};

} /* namespace efs */

#include "CarmenProcessor.hpp"

#endif /* EFS_CARMENPROCESSOR_H_ */
