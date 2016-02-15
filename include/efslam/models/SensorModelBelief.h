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

#ifndef EFS_SENSORMODELBELIEF_H_
#define EFS_SENSORMODELBELIEF_H_

#include <vector>
#include <boost/unordered_map.hpp>

#include "SensorModel.h"

#include "efslam/utils/Line.h"
#include "efslam/maps/cells/BeliefCell.h"


namespace efs {


template<int N>
class SensorModel< BeliefCell<N> > {
public:
	using	ScanMap = boost::unordered_map< Pointm<N>, BeliefCell<N> >;

								SensorModel( bool noLUTComputation = false );
								~SensorModel();

						void	setParamsConfig( bool noLUTComputation = false );

	/**
	 * @param pHit		end point of measurement in world coordinates
	 * @param scanLine	line from sensor origin to hit cell (up to usable range) in map coordinates
	 * @param hitIdx	index of the hit cell in scanLine, number >= scanLine.size() if hit cell is not part of scanLine (e.g. not in usable range)
	 * @param scanMap	cells updated in the current scan (only used in Belief model)
	 * @param map		the map
	 *
	 * @return	information gained in this step (if CALC_ENTROPY = 1)
	 */
	template<typename Map>
	inline				double 	inverseModel( const typename Map::WorldPoint &pHit, const Line<N> &scanLine, size_t hitIdx, ScanMap *scanMap, Map *map ) const;

	/**
	 * @param dist				distance between measurement and sensor origin (measured range)
	 * @param bestMu			distance between measurement and best matching cell
	 * @param bestMuMap			index of the best matching cell in the map
	 * @param sensorOrigin		origin of sensor in world coordinates
	 * @param sensorOriginMap 	origin of sensor in map coordinates
	 * @param map				the map
	 *
	 * @return log-likelihood
	 */
	template<typename Map>
	inline				double 	forwardModel( world_t dist, const typename Map::WorldPoint &bestMu, const typename Map::MapPoint &bestMuMap, const typename Map::WorldPoint &sensorOrigin, const typename Map::MapPoint &sensorOriginMap, const Map &map ) const;

	/**
	 * @param scanMap	cells updated in the current scan (only used in Belief model, container will be cleared)
	 * @param map		the map
	 *
	 * @return	information gained in this step (if CALC_ENTROPY = 1)
	 */
	template<typename Map>
	inline				double	integrate( ScanMap *scanMap, Map *map ) const;

	inline				double	sigmaSqr() const;
	inline				double	nullLogLikelihood() const;
	inline				bool	excludeNoHit() const;
	inline				int		cellsBehind() const;

private:
	double	m_beliefSigmaForward,
			m_beliefSigmaForwardSqr,
			m_beliefSigmaForwardFactor,
			m_beliefSigmaInverse,
			m_beliefSigmaInverseSqr,
			m_beliefSigmaInverseFactor,
			m_beliefRandom,
			m_beliefOccPrior;
	bool	m_beliefExcludeNoHit;
	int		m_beliefCellsBehind;

	world_t	m_maxRange,
			m_relevantDist;

#ifdef LUT_DEBUG
public:
#endif

	/**
	 * @param dist				distance between measurement and sensor origin (measured range)
	 * @param numCells			number of cells on the scan line
	 * @param cellBeliefs		belief of the cells with size = numCells
	 * @param distances			distances of the cells to the sensor origin with size = numCells
	 * @param i					current cell (only used in recursion, 0 for initial call)
	 * @param outerCell			whether this is the first cell (only used in recursion, true for initial call)
	 *
	 * @return log-likelihood
	 */
	inline	double					beliefLikelihood( world_t dist, int numCells, const BeliefFunction *cellBeliefs, const world_t *distances, int i = -1 ) const;

			void					computeBeliefLUT();
			void					computePlLUT();

	inline	const BeliefFunction&	getBelief( world_t dist, bool behind = false ) const;
	inline	double					getPl( world_t dist ) const;

	std::vector<BeliefFunction>	m_beliefLUT,		// values of the the inverse sensor model in the distance in front of hit cell
								m_beliefBehindLUT;	// values of the the inverse sensor model in the distance behind hit cell
	BeliefFunction				m_beliefZero,
								m_beliefBehindZero;

	std::vector<double>			m_plLUT;			// values of the forward sensor model in the distance between cell and measurement

	double						m_beliefDistScale,	// scale factor for distance discretization for belief LUT (controls its resolution)
								m_plDistScale;		// scale factor for distance discretization for pl LUT (controls its resolution)

	static constexpr size_t		MAX_SIZE = 5000;	// maximum number if entries in LUT (the actual size will be 2*MAX_SIZE, because of the two LUTs)
};

} /* namespace efs */


#include "SensorModelBelief.hpp"

#endif /* EFS_SENSORMODELBELIEF_H_ */
