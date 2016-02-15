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

#include "efslam/utils/Log.h"
#include "efslam/utils/Config.h"
#include "efslam/utils/NormalDistribution.h"


namespace efs {


/************************************
 * Bayes sensor model
 ************************************/

template<int N>
SensorModel< BayesCell<N> >::SensorModel( bool unused )
{
	setParamsConfig();
}


template<int N>
void
SensorModel< BayesCell<N> >::setParamsConfig() {
	m_bayesSigmaSqr 		=	Config::getDouble( "BAYES_SIGMA",  0.25 );
	m_bayesSigmaSqr 		*=	m_bayesSigmaSqr;
}


template<int N>
template<typename Map>
double
SensorModel< BayesCell<N> >::inverseModel( const typename Map::WorldPoint &pHit, const Line<N> &scanLine, size_t hitIdx, ScanMap *scanMap, Map *map ) const {
	static_assert( std::is_same< typename Map::CellType, BayesCell<N> >(), "Map cell type must be BayesCell<N>."  );

	double entropy = 0.0;

	for( size_t i = 0; i < scanLine.size(); i++ ) {
		BayesCell<N> &mapCell = map->cell( scanLine[i] );

#if CALC_ENTROPY
		entropy -= mapCell.entropy();
#endif

		if( i == hitIdx )
			mapCell.updateHit( pHit );
		else
			mapCell.updateNoHit();

#if CALC_ENTROPY
		entropy += mapCell.entropy();
#endif
	}

	return entropy;
}


template<int N>
template<typename Map>
double
SensorModel< BayesCell<N> >::forwardModel( world_t dist, const typename Map::WorldPoint &bestMu, const typename Map::MapPoint &bestMuMap, const typename Map::WorldPoint &sensorOrigin, const typename Map::MapPoint &sensorOriginMap, const Map &map ) const {
	static_assert( std::is_same< typename Map::CellType, BayesCell<N> >(), "Map cell type must be BayesCell<N>."  );
	return NormalDistribution::logPlFromSqr( bestMu.dot( bestMu ), m_bayesSigmaSqr );	// TODO: Save this in a LUT as well? (see Belief sensor model)
}


template<int N>
double
SensorModel< BayesCell<N> >::sigmaSqr() const {
	return m_bayesSigmaSqr;
}


template<int N>
double
SensorModel< BayesCell<N> >::nullLogLikelihood() const {
	return -0.5 / (2.0 * m_bayesSigmaSqr);
}


template<int N>
constexpr bool
SensorModel< BayesCell<N> >::excludeNoHit() const {
	return true;	// always exclude for bayes model
}


template<int N>
constexpr int
SensorModel< BayesCell<N> >::cellsBehind() const {
	return 0;
}


} /* namespace efs */
