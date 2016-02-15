/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping ScanMatcher class
 *  Copyright (c) 2004-2007, Giorgio Grisetti, Cyrill Stachniss, Wolfram Burgard
 *  Originally licensed under the Creative Commons (Attribution-NonCommercial-ShareAlike).
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

#ifndef EFS_MAP_SCANMATCHER_H_
#define EFS_MAP_SCANMATCHER_H_

#include <cstdint>
#include <vector>

#ifndef CALC_ENTROPY
#	define CALC_ENTROPY 0
#endif

#ifndef USE_MAP_CACHE
#	define USE_MAP_CACHE 0	// Cache seams to be slower because of dynamic allocations  TODO: Test this again with larger maps
#endif


#include "SensorModelBayes.h"
#include "SensorModelBelief.h"

#if USE_MAP_CACHE
#	include "maps/MapCache.h"
#endif

#include "efslam/utils/PoseBase.h"


namespace efs {


template<typename Scan, typename Map>
class MapScanMatcher {
public:
	static constexpr int 	Dimension		= Map::Dimension;
	using 					WorldPoint		= typename Map::WorldPoint;
	using 					MapPoint		= typename Map::MapPoint;
	using			 		CellType		= typename Map::CellType;
	using 					ScanConstPtr	= typename Scan::ConstPtr;

					MapScanMatcher( const PoseSE2 &sensorPose = PoseSE2() );

			void	setParamsConfig();
	inline	void	setSensorPose( const PoseSE2 &sensorPose, size_t sensorId = 0 );

	inline 	double	score( const ScanConstPtr &z, const PoseBase &pose, const Map &map, size_t sensorId = 0, double *logLikelihood = nullptr, uint32_t *count = nullptr ) const;
			double	registerScan( const ScanConstPtr &z, const PoseBase &pose, Map *map, size_t sensorId = 0 ) const;

			PoseSE2	optimize( const ScanConstPtr &z, const PoseSE2 &pose, const Map &map, size_t sensorId = 0, double *bestScore = nullptr ) const;

protected:
	template<typename MapT>
			double	scoreInternal( const ScanConstPtr &z, const PoseBase &pose, const MapT &map, size_t sensorId = 0, double *logLikelihood = nullptr, uint32_t *count = nullptr ) const;

	using SensorModelType 	= SensorModel< typename Map::CellType >;

	SensorModelType 			m_sensorModel;
	std::vector< PoseSE2 >		m_sensor2robot;
	std::vector< MapPoint >		m_kernelPoints;

	double	m_freeCellRatio,
			m_fullnessThreshold,
			m_scoreSigmaSqr;
	world_t	m_optAngularStep,
			m_optLinearStep,
			m_usableRange,
			m_maxRange,
			m_relevantDist;
	int		m_optIterations,
			m_likelihoodNumMeasurements;
	bool	m_likelihoodLikeScore;
};


} /* namespace efs */


#include "MapScanMatcher.hpp"


#endif /* EFS_MAP_SCANMATCHER_H_ */
