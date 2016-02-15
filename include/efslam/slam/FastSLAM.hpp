/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping GridSlamProcessor class
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

#include <algorithm>

#ifdef _OPENMP
#	include <omp.h>
#endif

#include "efslam/utils/Log.h"
#include "efslam/utils/Config.h"
#include "efslam/utils/Convenience.h"

namespace efs {


template<typename Pose, typename Map, bool ThreadSafe>
FastSLAM<Pose, Map, ThreadSafe>::FastSLAM() :
	m_numMeasurements( 0 ),
	m_linearDist( 0 ),
	m_angularDist( 0 ),
	m_likelihoodGain( 1 )
{
	l_inf( "Initializing FastSLAM" );
	setParamsConfig();
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::setParamsConfig() {
	m_linearDistThreshold	= Config::getDouble( "LINEAR_DIST_THRESHOLD", 	1.0 	);
	m_angularDistThreshold	= Config::getDouble( "ANGULAR_DIST_THRESHOLD", 	30.0 	);
	m_angularDistThreshold	= DEG2RAD( m_angularDistThreshold );
	m_minScore				= Config::getDouble( "MIN_SCORE",				0.0		);
	m_resampleThreshold		= Config::getDouble( "RESAMPLE_THRESHOLD",		0.5		);
	m_likelihoodGain		= Config::getDouble( "LIKELIHOOD_GAIN",			1.0		);

	typename Map::WorldPoint	min, max;
	min[0] 	= Config::getDouble( "MAP_MIN_X", -50 );
	max[0] 	= Config::getDouble( "MAP_MAX_X", 50 );
	min[1] 	= Config::getDouble( "MAP_MIN_Y", -50 );
	max[1] 	= Config::getDouble( "MAP_MAX_Y", 50 );
	world_t delta = Config::getDouble( "MAP_DELTA", 0.05 );

	// init particles
	int numParticles = Config::getInt( "NUM_PARTICLES", 50 );
	m_particles.reserve( numParticles );
	m_Neff = numParticles;

	Pose	initialPose;
	Map		map( Map::WorldPoint::Zero(), min, max, delta );
	auto 	rootNode = std::make_shared< TrajectoryNode<Pose> >( initialPose );

	for( size_t i = 0; i < numParticles; i++ ) {
		m_particles.push_back( ParticleType( initialPose, map, 1.0 / numParticles ) );
		m_particles[i].node = rootNode;
	}

	// print some infos
	l_inf( "Number of particles: " << numParticles );
	l_inf( "World size:          " << map.worldSize().transpose() << " [m]" );
	l_inf( "Map size:            " << map.mapSize().transpose() );
	l_inf( "World center:        " << map.center().transpose() << " [m]" );
	l_inf( "Map center:          " << map.world2map( map.center() ).transpose() );
	l_inf( "Patch size:          " << map.patchSize().transpose() << " -> " << map.patchSize().transpose().template cast<world_t>() * map.delta() << " [m]" );
	l_inf( "Number of patches:   " << map.numPatches().transpose() );
	l_inf( "Cell size:           " << map.delta() << " [m]" );
	l_inf( "Initial pose:        " << initialPose );
#ifdef _OPENMP
	l_inf( "Running parallel with OpenMP and a maximum of " << omp_get_max_threads() << " threads." );
#endif
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::setSensorPose( const PoseSE2 &sensorPose, size_t sensorId ) {
	m_scanMatcher.setSensorPose( sensorPose, sensorId );
}


/**********************************
 * Processing
 **********************************/

template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::processOdom( double linearMove, double angularMove ) {
	m_mutex.lock();

	l_dbg( UNDERLINE "Processing odometry: " << linearMove << "m  " << RAD2DEG( angularMove ) << "deg" NORMAL );

	m_stopWatch.reset();
#	ifdef _OPENMP
#	pragma omp parallel for
#	endif
	for( size_t i = 0; i < m_particles.size(); i++ ) {
		m_particles[i].pose = m_motionModel.draw( m_particles[i].pose, linearMove, angularMove );
	}

	m_linearDist 	+= fabs( linearMove );
	m_angularDist	+= fabs( ANGLE_NORMALIZE( angularMove ) );

	l_dbg( " - Time needed:             " << m_stopWatch.timePast() );

	m_mutex.unlock();
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::processOdom( const Pose &oldPose, const Pose &newPose ) {
	m_mutex.lock();

	l_dbg( UNDERLINE "Processing odometry: " << oldPose << " -> " << newPose << NORMAL );

	m_stopWatch.reset();
#	ifdef _OPENMP
#	pragma omp parallel for
#	endif
	for( size_t i = 0; i < m_particles.size(); i++ ) {
		m_particles[i].pose = m_motionModel.draw( m_particles[i].pose, oldPose, newPose );
	}

	Pose diff = newPose - oldPose;
	m_linearDist 	+= diff.posNorm();
	m_angularDist	+= diff.angNorm();

	l_dbg( " - Time needed:             " << m_stopWatch.timePast() );

	m_mutex.unlock();
}


template<typename Pose, typename Map, bool ThreadSafe>
bool
FastSLAM<Pose, Map, ThreadSafe>::scanRequired() const {
	return !m_numMeasurements || m_linearDist > m_linearDistThreshold || m_angularDist > m_angularDistThreshold;
}


template<typename Pose, typename Map, bool ThreadSafe>
bool
FastSLAM<Pose, Map, ThreadSafe>::processScan( const ScanConstPtr &z, size_t sensorId, bool forceProcessing ) {
	bool processed = false;

	if( forceProcessing || scanRequired() ) {
		m_mutex.lock();
		l_inf( UNDERLINE "Processing scan " << m_numMeasurements << ": " << z->size() << " points" NORMAL );
		auto startTime = m_stopWatch.now();

		// Not the first measurement?
		if( m_numMeasurements ) {

			double 				sumScore 	= 0;
			uint32_t			sumCount	= 0;

			// Scan matching
			l_inf( " - Scan matching" );

			m_stopWatch.reset();
#			ifdef _OPENMP
#			pragma omp parallel for reduction(+:sumScore,sumCount)
#			endif
			for( size_t i = 0; i < m_particles.size(); i++ ) {
				double		score,
							logLikelihood;
				uint32_t	count;

				Pose corrected = m_scanMatcher.optimize( z, m_particles[i].pose, m_particles[i].map, sensorId, &score );

				if( score > m_minScore ) {
					m_particles[i].pose = corrected;
				} else {
					l_wrn( "Scan matching failed for particle " << i << " with score " << score << " (must be >" << m_minScore << ")! Using odometry only." );
				}

				m_scanMatcher.score( z, m_particles[i].pose, m_particles[i].map, sensorId, &logLikelihood, &count );
				m_particles[i].logWeight 	+= logLikelihood;
				m_particles[i].logWeightSum	+= logLikelihood;
				sumScore 					+= score;
				sumCount					+= count;
			}

			//updateTreeWeights( &weights, &m_Neff );
			updateNormalizedWeightsAndNeff();

			l_inf( "   Average score:           " << sumScore / m_particles.size() );
			l_inf( "   Average used measurem.:  " << (double) sumCount / m_particles.size() );
			l_inf( "   Neff:                    " << m_Neff );
			l_inf( "   Time needed:             " << m_stopWatch.timePast() );

			resampleAndRegister( z, sensorId );

		// First measurement
		} else {
			l_inf( " - Registering first scan" );
			registerScan( z, m_particles, sensorId );
		}

		m_linearDist	= 0;
		m_angularDist	= 0;
		m_numMeasurements++;

		processed = true;
		l_inf( " - Total time needed:       " << m_stopWatch.timePast( startTime ) );

		m_mutex.unlock();
	}

	return processed;
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::registerScan( const ScanConstPtr &z, ParticleVector &particles, size_t sensorId ) const {
	m_stopWatch.reset();
#	ifdef _OPENMP
#	pragma omp parallel for
#	endif
	for( size_t i = 0; i < particles.size(); i++ ) {
		m_scanMatcher.registerScan( z, particles[i].pose, &particles[i].map, sensorId );
	}
	l_inf( "   Time needed:             " << m_stopWatch.timePast() );
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::registerScan( const ScanConstPtr &z, ParticlePtrVector &particles, size_t sensorId ) const {
	m_stopWatch.reset();
#	ifdef _OPENMP
#	pragma omp parallel for
#	endif
	for( size_t i = 0; i < particles.size(); i++ ) {
		m_scanMatcher.registerScan( z, particles[i]->pose, &particles[i]->map, sensorId );
	}
	l_inf( "   Time needed:             " << m_stopWatch.timePast() );
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::resampleAndRegister( const ScanConstPtr &z, size_t sensorId ) {
	if( m_Neff < m_resampleThreshold * m_particles.size() ) {
		// Resampling required
		l_inf( " - Resampling" );
		m_stopWatch.reset();
		auto indexes = resampleIndexes();

		// Collect sampled and deleted particles
		ParticlePtrVector	uniqueParticles; // TODO: Use a list here?

		// get samples particles and reset weights
		for( size_t i = 0, size = indexes.size(); i < size; i++ ) {
			// each sampled particle only once
			if( i == 0 || indexes[i] != indexes[i-1] ) {
				ParticleType *p = &m_particles[indexes[i]];

				p->logWeight	= 0;
				p->weight		= 1.0 / m_particles.size();

				uniqueParticles.push_back( p );
			}
		}
		l_inf( "   Unique particles:        " << uniqueParticles.size() );
		l_inf( "   Time needed:             " << m_stopWatch.timePast() );

		// Registering scan only for unique particles
		l_inf( " - Registering scan" );
		registerScan( z, uniqueParticles, sensorId );

		// Pruning
		if( Map::Dimension > 2 ) {
			l_inf( " - Pruning maps" );
			m_stopWatch.reset();
			uint32_t	sumBranches = 0;
#			ifdef _OPENMP
#			pragma omp parallel for reduction(+:sumBranches)
#			endif
			for( size_t i = 0; i < uniqueParticles.size(); i++ ) {
				sumBranches += uniqueParticles[i]->map.prune();
			}
			l_inf( "   Average pruned branches: " << ((double) sumBranches / uniqueParticles.size()) );
			l_inf( "   Time needed:             " << m_stopWatch.timePast() );
		}

		// Copy particles according to there actual sample count
		ParticleVector	newParticles;
		newParticles.reserve( indexes.size() );
		size_t j = 0;
		for( size_t i = 0; i < indexes.size(); i++ ) {
			if( i != 0 && indexes[i] != indexes[i-1] )
				j++;

			newParticles.push_back( *uniqueParticles[j] );
		}

		// delete particles
		m_particles.clear();

		// Swap vectors
		m_particles.swap( newParticles );

		// Update Neff
		m_Neff = m_particles.size();
	} else {
		// No resampling required
		l_inf( " - Registering scan" );
		registerScan( z, m_particles, sensorId );
	}

	// create the new generation of tree nodes
	createTreeNodes();
}


template<typename Pose, typename Map, bool ThreadSafe>
std::vector<size_t>
FastSLAM<Pose, Map, ThreadSafe>::resampleIndexes() const {
	// Uniform resampling
    // Similar to Probabilistic Robotics, page 110, table 4.4

	std::random_device						rd;
	std::mt19937							generator( rd() );
	std::uniform_real_distribution<double>	distribution( 0.0, 1.0 );

	size_t	n		= m_particles.size(),
			j		= 0;
	double 	cweight	= 0;

	std::vector<size_t>	indexes( n );

	// cumulative weights (for interval computation, is set to zero again later)
	for( const auto &p : m_particles )
		cweight += p.weight;

	// interval
	double interval = cweight / n;

	// initial target weight
	double target = interval * distribution( generator );

	// draw samples
	cweight	= 0;
	for( size_t i = 0; i < n; i++) {
		cweight += m_particles[i].weight;
		while( cweight > target ) {
			indexes[j++] =  i;
			target 		 += interval;
		}
	}

	assert( j == m_particles.size() );
	return indexes;
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::updateNormalizedWeightsAndNeff() {
	double	gain	= 1. / (m_likelihoodGain * m_particles.size() ),
			lmax	= -std::numeric_limits<double>::max(),
			wcum	= 0;

	for( const auto &p : m_particles )
		if( p.logWeight > lmax )
			lmax = p.logWeight;

	for( auto &p : m_particles ) {
		p.weight	=	exp( gain * (p.logWeight - lmax) );
		wcum		+=	p.weight;
	}

	m_Neff = 0;
	for( auto &p : m_particles ) {
		p.weight	/= wcum;
		m_Neff		+= SQR( p.weight );
	}

	m_Neff = 1.0 / m_Neff;
}


/**********************************
 * Getter
 **********************************/

template<typename Pose, typename Map, bool ThreadSafe>
size_t
FastSLAM<Pose, Map, ThreadSafe>::bestParticleIdx( bool lock ) const {
	if( lock )
		m_mutex.lock();

	size_t	bestIdx		= 0;
	double	bestWeight	= m_particles[0].logWeightSum;

	for( size_t i = 1; i < m_particles.size(); i++ )
		if( bestWeight < m_particles[i].logWeightSum ) {
			bestIdx		= i;
			bestWeight	= m_particles[i].logWeightSum;
		}

	if( lock )
		m_mutex.unlock();

	return bestIdx;
}


template<typename Pose, typename Map, bool ThreadSafe>
const typename FastSLAM<Pose, Map, ThreadSafe>::ParticleType&
FastSLAM<Pose, Map, ThreadSafe>::particle( size_t idx ) const {
	return m_particles[idx];
}


template<typename Pose, typename Map, bool ThreadSafe>
const Pose&
FastSLAM<Pose, Map, ThreadSafe>::pose( size_t idx ) const {
	return m_particles[idx].pose;
}


template<typename Pose, typename Map, bool ThreadSafe>
typename FastSLAM<Pose, Map, ThreadSafe>::Covariances
FastSLAM<Pose, Map, ThreadSafe>::covariances( const std::vector<size_t> &timePoints, bool lock ) const {
	if( !timePoints.size() )
		return Covariances();

	if( lock )
		m_mutex.lock();

	using Samples	= std::vector< typename Eigen::Matrix<double, Pose::VectorSize, 1> >;
	using Weights	= std::vector<double>;

	Covariances 		res;
	TrajectoryVector	trajVec = trajectories( false );	// TODO: calculation can be optimized by collecting the poses on the fly and take tree structure into account
	Weights				weights;
	//size_t				bestIdx = bestParticleIdx( false );

	weights.resize( m_particles.size() );
	res.resize( timePoints.size() );

	for( size_t i = 0; i < m_particles.size(); i++ )
		weights[i] = m_particles[i].weight;

#	ifdef _OPENMP
#	pragma omp parallel for
#	endif
	for( size_t t = 0; t < timePoints.size(); t++ ) {
		Samples samples;
		samples.resize( m_particles.size() );

		for( size_t i = 0; i < m_particles.size(); i++ )
			samples[i] = trajVec[i][timePoints[t]].toVector().template cast<double>();

		auto mean = NormalDistribution::mean<Pose::VectorSize, Samples, Weights>( samples, weights );
		//auto mean = trajVec[bestIdx][timePoints[t]].toVector().template cast<double>();

		auto cov = NormalDistribution::cov<Pose::VectorSize, Samples, Weights>( mean, samples, weights );
		res[t] = cov;
	}

	if( lock )
		m_mutex.unlock();

	return res;
}


template<typename Pose, typename Map, bool ThreadSafe>
const Map&
FastSLAM<Pose, Map, ThreadSafe>::map( size_t idx, bool lock ) const {
	// lock can be safely ignored
	return m_particles[idx].map;
}


template<typename Pose, typename Map, bool ThreadSafe>
double
FastSLAM<Pose, Map, ThreadSafe>::weight( size_t idx ) const {
	return m_particles[idx].weight;
}


template<typename Pose, typename Map, bool ThreadSafe>
typename FastSLAM<Pose, Map, ThreadSafe>::TrajectoryVector
FastSLAM<Pose, Map, ThreadSafe>::trajectories( bool lock ) const {
	return trajectories( 0, lock );
}


template<typename Pose, typename Map, bool ThreadSafe>
typename FastSLAM<Pose, Map, ThreadSafe>::TrajectoryVector
FastSLAM<Pose, Map, ThreadSafe>::trajectories( size_t maxDepth, bool lock ) const {
	TrajectoryVector res( m_particles.size() );

	if( lock )
		m_mutex.lock();

#	ifdef _OPENMP
#	pragma omp parallel for
#	endif
	for( size_t i = 0; i < m_particles.size(); i++ ) {
		res[i] = trajectory( i, maxDepth, false );
	}

	if( lock )
		m_mutex.unlock();

	return res;
}


template<typename Pose, typename Map, bool ThreadSafe>
typename FastSLAM<Pose, Map, ThreadSafe>::Trajectory
FastSLAM<Pose, Map, ThreadSafe>::trajectory( size_t idx, bool lock ) const {
	return trajectory( idx, 0, lock );
}


template<typename Pose, typename Map, bool ThreadSafe>
typename FastSLAM<Pose, Map, ThreadSafe>::Trajectory
FastSLAM<Pose, Map, ThreadSafe>::trajectory( size_t idx, size_t maxDepth, bool lock ) const {
	if( lock )
		m_mutex.lock();

	Trajectory	res;
	res.reserve( maxDepth ? maxDepth : m_numMeasurements );

	for( TrajectoryNode<Pose> *node = m_particles[idx].node.get(); node && (!maxDepth || res.size() < maxDepth); node = node->parent.get() ) {
		res.push_back( node->pose );
	}

	std::reverse( res.begin(), res.end() );

	if( lock )
		m_mutex.unlock();

	return res;
}


template<typename Pose, typename Map, bool ThreadSafe>
void
FastSLAM<Pose, Map, ThreadSafe>::createTreeNodes() {
	// Do not run this in parallel, because multiple nodes may have the same parent
	for( auto &p : m_particles )
		p.newNode();
}


} /* namespace efs */

