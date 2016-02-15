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

#include <unordered_map>

#include "efslam/maps/cells/BeliefCell.h"
#include "efslam/maps/cells/BayesCell.h"

#include "efslam/utils/Log.h"
#include "efslam/utils/Config.h"
#include "efslam/utils/PointIterator.h"
#include "efslam/utils/PointCrossIterator.h"


namespace efs {


template<typename Scan, typename Map>
MapScanMatcher<Scan, Map>::MapScanMatcher( const PoseSE2 &sensorPose ) :
	m_sensorModel( true )
{
	m_sensor2robot.push_back( sensorPose );
	setParamsConfig();
}


template<typename Scan, typename Map>
void
MapScanMatcher<Scan, Map>::setParamsConfig() {
	m_freeCellRatio				= Config::getDouble( 	"FREE_CELL_RATIO", 				sqrt( 2.0 ) );
	m_usableRange				= Config::getDouble( 	"USABLE_RANGE", 				15.0 	);
	m_maxRange					= Config::getDouble( 	"MAX_RANGE", 					80.0 	);
	m_fullnessThreshold			= Config::getDouble( 	"FULLNESS_THRESHOLD", 			0.1 	);
	m_scoreSigmaSqr				= Config::getDouble(	"SCORE_SIGMA", 					0.25 	);
	m_scoreSigmaSqr				*= m_scoreSigmaSqr;
	m_sensorModel.setParamsConfig();
	m_relevantDist				= Config::getDouble( 	"RELEVANT_DIST",				0.0		);
	m_optAngularStep			= Config::getDouble( 	"OPT_ANGULAR_STEP", 			3.0 	);
	m_optAngularStep			= DEG2RAD( m_optAngularStep );
	m_optLinearStep				= Config::getDouble( 	"OPT_LINEAR_STEP", 				.05		);
	m_optIterations				= Config::getInt( 		"OPT_ITERATIONS", 				5		);
	m_likelihoodNumMeasurements	= Config::getInt( 		"LIKELIHOOD_NUM_MEASUREMENTS",	0 		);
	m_likelihoodLikeScore		= Config::getInt( 		"LIKELIHOOD_LIKE_SCORE",		0		);


	int		kernelSize	= Config::getInt( "KERNEL_SIZE", 1 );
	bool	kernelCross = false;

	kernelCross = Config::getInt( "KERNEL_CROSS_2D", 0 );

	// pre-compute points in kernel
	if( kernelCross ) {
		for( PointCrossIterator<Dimension> iter( MapPoint::Ones() * -kernelSize, MapPoint::Ones() * (kernelSize+1) ); iter; ++iter )
			m_kernelPoints.push_back( *iter );
	} else {
		for( PointIterator<Dimension> iter( MapPoint::Ones() * -kernelSize, MapPoint::Ones() * (kernelSize+1) ); iter; ++iter )
			m_kernelPoints.push_back( *iter );
	}
}


template<typename Scan, typename Map>
void
MapScanMatcher<Scan, Map>::setSensorPose( const PoseSE2 &sensorPose, size_t sensorId ) {
	if( sensorId >= m_sensor2robot.size() )
		m_sensor2robot.resize( sensorId + 1 );

	m_sensor2robot[sensorId] = sensorPose;
}

template<typename Scan, typename Map>
double
MapScanMatcher<Scan, Map>::score( const ScanConstPtr &z, const PoseBase &pose, const Map &map, size_t sensorId, double *logLikelihood, uint32_t *count ) const {
#if USE_MAP_CACHE
	return scoreInternal( z, pose, MapCacheRead<Map>( &map ), sensorId, logLikelihood, count );
#else
	return scoreInternal( z, pose, map, sensorId, logLikelihood, count );
#endif
}


template<typename Scan, typename Map>
template<typename MapT>
double
MapScanMatcher<Scan, Map>::scoreInternal( const ScanConstPtr &z, const PoseBase &pose, const MapT &map, size_t sensorId, double *logLikelihood, uint32_t *count ) const {
	double 	score 			= 0,
			freeDelta		= map.delta() * m_freeCellRatio,
			noHit			= m_sensorModel.nullLogLikelihood();
	bool	excludeNoHit	= m_sensorModel.excludeNoHit();

	if( logLikelihood )
		*logLikelihood = 0;

	if( count )
		*count = 0;

	assert( sensorId < m_sensor2robot.size() );

	auto sensor2world	=	pose.toPoseSE<Dimension>();
	sensor2world 		*=	m_sensor2robot[sensorId];

	WorldPoint	sensorOrigin	= sensor2world * WorldPoint::Zero();	// the origin of our sensor is 0 0 (0) in sensor coordinates
	MapPoint 	sensorOriginMap = map.world2map( sensorOrigin );

	// Calculate number of points to skip
	int skipPoints = 0;
	if( logLikelihood && !m_likelihoodLikeScore && m_likelihoodNumMeasurements ) {
		skipPoints = z->size() / m_likelihoodNumMeasurements - 1;
		if( skipPoints < 0 )
			skipPoints = 0;
	}

	size_t i = 0;
	for( const auto &z_i : *z ) {
		// Skip m_scoreSkipPoints points or, in likelihood computation, so many that
		// the total number is m_likelihoodNumMeasurements before considering one for the calculation
		i++;
		if( (i % (skipPoints+1)) != 0 )
			continue;

		world_t dist = z_i.norm();
		if( dist == 0 || dist > m_usableRange || dist > m_maxRange )
			continue;

		// Hit cell
		WorldPoint	pointHitSensor( z_i );
		WorldPoint	pointHit 		= sensor2world * pointHitSensor;
		MapPoint	pointHitMap		= map.world2map( pointHit );

		// Cell before hit cell
		WorldPoint	pointFree		= sensor2world * ( pointHitSensor * ((dist - freeDelta) / dist) );
		MapPoint	pointFreeMap	= map.world2map( pointFree );

		bool 		found 		= false;
		WorldPoint	bestMu		= WorldPoint::Zero();
		MapPoint	bestMuMap	= pointHitMap;

		for( const auto &point : m_kernelPoints ) {
			MapPoint		hitIdx 		= pointHitMap  + point,
							freeIdx		= pointFreeMap + point;

			const CellType	*hitCell	= &map.cell( hitIdx ),
							*freeCell	= &map.cell( freeIdx );

			if( hitCell->fullness() > m_fullnessThreshold && freeCell->fullness() < m_fullnessThreshold ) {
				WorldPoint mu = pointHit - hitCell->mean();
				if( !found || mu.dot( mu ) < bestMu.dot( bestMu ) ) {
					bestMu		= mu;
					bestMuMap	= hitIdx;
					found		= true;
				}
			}
		}

		if( found ) {
			score += NormalDistribution::plFromSqr( bestMu.dot( bestMu ), m_scoreSigmaSqr );
			if( count )
				(*count)++;
		}

		if( logLikelihood ) {
			if( !excludeNoHit || found )
				*logLikelihood += m_sensorModel.forwardModel( dist, bestMu, bestMuMap, sensorOrigin, sensorOriginMap, map );
			else
				*logLikelihood += noHit;
		}
	}

	return score;
}


template<typename Scan, typename Map>
double
MapScanMatcher<Scan, Map>::registerScan( const ScanConstPtr &z, const PoseBase &pose, Map *map_, size_t sensorId ) const {
#if USE_MAP_CACHE
	MapCacheWrite<Map>	cachedMap( map_ );
	MapCacheWrite<Map>	*map( &cachedMap );
#else
	Map 				*map( map_ );
#endif

	double entropy = 0.0;

	typename SensorModelType::ScanMap	scanMap;

	assert( sensorId < m_sensor2robot.size() );

	auto sensor2world	=	pose.toPoseSE<Dimension>();
	sensor2world 		*=	m_sensor2robot[sensorId];

	WorldPoint	sensorOrigin	= sensor2world * WorldPoint::Zero();	// the origin of our sensor is 0 0 (0) in sensor coordinates
	MapPoint 	sensorOriginMap = map->world2map( sensorOrigin );

	for( const auto &z_i : *z ) {
		world_t 	dist 	= z_i.norm();
		bool		usable	= (dist <= m_usableRange);

		if( !usable && m_relevantDist > 0.0 )
			continue;

		if( dist > m_maxRange )
			continue;

		// Hit cell
		WorldPoint	pointHitSensor( z_i );
		WorldPoint	pointHit	= sensor2world * pointHitSensor;
		WorldPoint	pointEnd	= usable ? pointHit : sensor2world * ( pointHitSensor * (m_usableRange / dist) );
		MapPoint	pointEndMap	= map->world2map( pointEnd );

		//l_dbg( "sensorOriginMap " << sensorOriginMap.transpose() << "   pHit " << pHit.transpose() << "   pEnd " << pEnd.transpose() << "   pEndMap " << pEndMap.transpose() );

		MapPoint	pointOriginMap( sensorOriginMap );
		if( m_relevantDist > 0.0 && dist > m_relevantDist ) {
			pointOriginMap = pointEndMap - ( (pointHit - sensorOrigin) * ( m_relevantDist / (dist * map->delta()) ) ).template cast<map_t>();
		}

		Line<Dimension>		scanLine( pointOriginMap, pointEndMap ); 		// all cells from laser pose to measurement
		size_t				hitIdx	= scanLine.size() + (usable ?  -1 : 0);	// if not usable, hitIdx is outside of scanLine

		// cells behind measurement (only for belief maps)
		if( usable && m_sensorModel.cellsBehind() ) {
			MapPoint pointBehindMap	= pointEndMap + ( (pointHit - sensorOrigin) * ( m_sensorModel.cellsBehind() / dist ) ).template cast<map_t>();
			scanLine.extend( pointBehindMap ); // extend the current scan line to the point behind the measurement
		}

		entropy += m_sensorModel.inverseModel( pointHit, scanLine, hitIdx, &scanMap, map );
	}

	entropy += m_sensorModel.integrate( &scanMap, map );

	return entropy;
}


template<typename Scan, typename Map>
PoseSE2
MapScanMatcher<Scan, Map>::optimize( const ScanConstPtr &z, const PoseSE2 &pose, const Map &map_, size_t sensorId, double *bestScoreOut ) const {
#if USE_MAP_CACHE
	const MapCacheRead<Map>	map( &map_ );
#else
	const Map				&map( map_ );
#endif

	world_t		angStep 		= m_optAngularStep,
				linStep			= m_optLinearStep;
	PoseSE2		currentPose 	= pose;
	double		currentScore	= scoreInternal( z, currentPose, map, sensorId ),
				bestScore		= -1;
	int			refinement		= 0;

	enum Move {
		Front, Back, Left, Right, TurnLeft, TurnRight, Done
	};

	assert( currentScore > bestScore );

	// until best score is found and not max iterations reached
	while( currentScore > bestScore || refinement < m_optIterations ) {
		if( bestScore >= currentScore ) {
			refinement++;
			angStep *= .5;
			linStep *= .5;
		}

		bestScore				= currentScore;
		PoseSE2	bestLocalPose 	= currentPose,
				localPose;

		Move move = Front;
		while( move != Done ) {
			localPose = currentPose;

			switch( move ) {
				case Front:
					localPose.x() += linStep;
					move = Back;
					break;
				case Back:
					localPose.x() -= linStep;
					move = Left;
					break;
				case Left:
					localPose.y() -= linStep;
					move = Right;
					break;
				case Right:
					localPose.y() += linStep;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.phi() += angStep;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.phi() -= angStep;
					move = Done;
					break;
				default:
					break;
			}

			double odoGain = 1.0;
			/*
			// m_angularOdometryReliability and m_linearOdometryReliability are set to 0.0 in GMapping by default
			if (m_angularOdometryReliability > 0.) {
				world_t dphi 	= 	pose.phi - localPose.phi;
				dphi			= 	ANGLE_NORMALIZE( dphi );
				dphi			*= 	dphi;
				odoGain 		*= 	exp( -m_angularOdometryReliability * dphi );
			}
			if (m_linearOdometryReliability > 0.) {
				Point2w	dpos	= 	pose.pos - localPose.pos;
				world_t drho 	= 	dpos.squaredNorm();
				odoGain 		*=	exp( -m_linearOdometryReliability * drho );
			}
			*/

			double localScore = odoGain * scoreInternal( z, localPose, map, sensorId );

			if( localScore > currentScore ) {
				currentScore	= localScore;
				bestLocalPose	= localPose;
			}
		};

		currentPose = bestLocalPose;
	};


	if( bestScoreOut )
		*bestScoreOut = bestScore;

	currentPose.phi() = ANGLE_NORMALIZE( currentPose.phi() );
	return currentPose;
}


} /* namespace efs */

