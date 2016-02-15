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

#ifndef EFS_FASTSLAM_H_
#define EFS_FASTSLAM_H_

#include <vector>
#include <list>

#include "SLAMTypes.h"
#include "Particle.h"
#include "TrajectoryNode.h"

#include "efslam/utils/ConditionalMutex.h"
#include "efslam/utils/Stopwatch.h"
#include "efslam/utils/Scan.h"

#include "efslam/models/MapScanMatcher.h"
#include "efslam/models/MotionModel.h"


namespace efs {

template<typename Pose, typename Map, bool ThreadSafe = true>
class FastSLAM {
public:
	using PoseType			= Pose;
	using MapType			= Map;
	using ParticleType		= Particle<Pose, Map>;
	using TrajectoryNodePtr = typename TrajectoryNode<Pose>::Ptr;
	using Trajectory		= std::vector< Pose >;
	using TrajectoryVector	= std::vector< Trajectory >;
	using ParticleVector	= std::vector< ParticleType >;
	using ParticlePtrVector	= std::vector< ParticleType* >;
	using PointType			= Pointw<Map::Dimension>;
	using ScanType			= Scan< PointType >;
	using ScanConstPtr		= typename ScanType::ConstPtr;
	using Covariance		= Eigen::Matrix<double, Pose::DOF, Pose::DOF>;
	using Covariances		= std::vector<Covariance>;

								FastSLAM();

			void				setParamsConfig();
	inline	void				setSensorPose( const PoseSE2 &sensorPose, size_t sensorId = 0 );

			void				processOdom( double linearMove, double angularMove );
			void				processOdom( const Pose &oldPose, const Pose &newPose );

			bool				processScan( const ScanConstPtr &z, size_t sensorId = 0, bool forceProcessing = false ); // forceProcessing = true processes the scan, even if scanRequired() is false. This is useful when multiple sensors are used and the process handling is done by the surrounding task.
	inline	bool				scanRequired() const; //  returns whether the next scan would be processed

	inline	void				lock() const		{ m_mutex.lock(); 	}
	inline	void				unlock() const		{ m_mutex.unlock();	}
	inline	bool				try_lock() const	{ return m_mutex.try_lock(); }

	inline	size_t				bestParticleIdx( bool lock = true ) const;
	inline	const ParticleType&	particle( size_t idx ) const;
	inline	const Pose&			pose( size_t idx ) const;
	inline	Covariances			covariances( const std::vector<size_t> &timePoints, bool lock = true ) const;
	inline	const Map&			map( size_t idx, bool lock = true ) const;
	inline	double				weight( size_t idx ) const;
	inline	TrajectoryVector	trajectories( bool lock = true ) const;
	inline	TrajectoryVector	trajectories( size_t maxDepth, bool lock = true ) const;
	inline	Trajectory			trajectory( size_t idx, bool lock = true ) const;
	inline	Trajectory			trajectory( size_t idx, size_t maxDepth, bool lock = true ) const;
	inline	double				Neff() const 						{ return m_Neff; }
	inline	size_t				numParticles() const				{ return m_particles.size(); }
	inline	uint64_t			numMeasurements() const				{ return m_numMeasurements; }

	inline	static constexpr SLAMType type() {	return SLAM_FAST;	};

protected:
	inline	void				registerScan( const ScanConstPtr &z, ParticleVector &particles, size_t sensorId = 0 ) const;
	inline	void				registerScan( const ScanConstPtr &z, ParticlePtrVector &particles, size_t sensorId = 0 ) const;
			void				resampleAndRegister( const ScanConstPtr &z, size_t sensorId = 0 );
			std::vector<size_t>	resampleIndexes() const;

			void				createTreeNodes();
	inline	void				updateNormalizedWeightsAndNeff();

	MapScanMatcher< ScanType, Map >	m_scanMatcher;
	MotionModel< Pose >				m_motionModel;

	ParticleVector		m_particles;
	double				m_Neff;

    uint64_t	m_numMeasurements;

    world_t		m_linearDist,
    			m_angularDist,
    			m_linearDistThreshold,
    			m_angularDistThreshold;
    double		m_minScore,
    			m_resampleThreshold,
    			m_likelihoodGain;

    mutable Stopwatch	m_stopWatch;
    mutable ConditionalMutex<ThreadSafe>	m_mutex;
};

} /* namespace efs */

#include "FastSLAM.hpp"

#endif /* EFS_FASTSLAM_H_ */
