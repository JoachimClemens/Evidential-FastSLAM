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
 * Belief sensor model
 ************************************/

template<int N>
SensorModel< BeliefCell<N> >::SensorModel( bool noLUTComputation )
{
	setParamsConfig( noLUTComputation );
}


template<int N>
void
SensorModel< BeliefCell<N> >::setParamsConfig( bool noLUTComputation ) {
	m_beliefSigmaForward		= Config::getDouble(	"BELIEF_SIGMA_FORWARD", 		0.025 );
	m_beliefSigmaForwardSqr		= m_beliefSigmaForward*m_beliefSigmaForward;
	m_beliefSigmaForwardFactor	= Config::getDouble(	"BELIEF_SIGMA_FORWARD_FACTOR",	2.0 );
	m_beliefSigmaInverse		= Config::getDouble(	"BELIEF_SIGMA_INVERSE", 		0.025 );
	m_beliefSigmaInverseSqr		= m_beliefSigmaInverse*m_beliefSigmaInverse;
	m_beliefSigmaInverseFactor	= Config::getDouble(	"BELIEF_SIGMA_INVERSE_FACTOR",	3.0 );

	m_beliefRandom				= Config::getDouble(	"BELIEF_RANDOM", 				0.2 );
	m_beliefOccPrior			= Config::getDouble(	"BELIEF_OCC_PRIOR", 			0.4 );
	m_beliefExcludeNoHit		= Config::getInt( 		"BELIEF_EXCLUDE_NO_HIT", 		0 );
	m_beliefCellsBehind			= Config::getInt( 		"BELIEF_CELLS_BEHIND",			3 );

	m_relevantDist				= Config::getDouble( 	"RELEVANT_DIST",				0.0		);
	m_maxRange					= Config::getDouble( 	"MAX_RANGE", 					15.0 );

	if( !noLUTComputation ) {
		computeBeliefLUT();
		computePlLUT();
	}
}


template<int N>
SensorModel< BeliefCell<N> >::~SensorModel() {
	// Nothing to do here
}


template<int N>
template<typename Map>
double
SensorModel< BeliefCell<N> >::inverseModel( const typename Map::WorldPoint &pHit, const Line<N> &scanLine, size_t hitIdx, ScanMap *scanMap, Map *map ) const {
	static_assert( std::is_same< typename Map::CellType, BeliefCell<N> >(), "Map cell type must be BeliefCell<N>."  );

	for( size_t i = 0; i < scanLine.size(); i++ ) {
		// Update scanMap using Dempster's rule
		if( i == hitIdx ) {
			(*scanMap)[scanLine[i]].updateHit( getBelief( 0.0 ), pHit ); // we want the hit cell directly on the measurement
		} else {
			typename Map::WorldPoint cellWorld	= map->map2world( scanLine[i] );
			const BeliefFunction 	&bf 		= getBelief( (cellWorld - pHit).norm(), (i > hitIdx) );

			(*scanMap)[scanLine[i]].updateNoHit( bf, (i > hitIdx) );	// i > hitIdx -> cell behind measurement
		}
	}

	return 0.0;
}


template<int N>
template<typename Map>
double
SensorModel< BeliefCell<N> >::integrate( ScanMap *scanMap, Map *map ) const {
	static_assert( std::is_same< typename Map::CellType, BeliefCell<N> >(), "Map cell type must be BeliefCell<N>."  );
	static_assert( std::is_same< typename Map::MapPoint, Pointm<N> >(), "Map MapPoint type must be Pointm<N>."  );

	double entropy = 0.0;

	// Integrate scan map using conjunctive rule
	for( const auto &cell : *scanMap ) {
		BeliefCell<N> &mapCell = map->cell( cell.first );

#if CALC_ENTROPY
		entropy -= mapCell.entropy();
#endif

		mapCell.integrate( cell.second );

#if CALC_ENTROPY
		entropy += mapCell.entropy();
#endif

		//std::cout << "(" << cell.first.transpose() << ") " << mapCell << std::endl;
	}

	scanMap->clear();

	return entropy;
}


template<int N>
template<typename Map>
double
SensorModel< BeliefCell<N> >::forwardModel( world_t dist, const typename Map::WorldPoint &bestMu, const typename Map::MapPoint &bestMuMap, const typename Map::WorldPoint &sensorOrigin, const typename Map::MapPoint &sensorOriginMap, const Map &map ) const {
	static_assert( std::is_same< typename Map::CellType, BeliefCell<N> >(), "Map cell type must be BeliefCell<N>."  );
	static_assert( std::is_same< typename Map::MapPoint, Pointm<N> >(), "Map MapPoint type must be Pointm<N>."  );

	// Direction from sensor origin to measurement in world coordinates
	typename Map::WorldPoint direction = bestMuMap.template cast<world_t>() - sensorOriginMap.template cast<world_t>();
	direction.normalize();

	typename Map::MapPoint	originMap( sensorOriginMap );
	if( m_relevantDist > 0.0 && dist > m_relevantDist ) {
		originMap = bestMuMap - (direction * (m_relevantDist / map.delta())).template cast<map_t>();
	}

	Line<N>	line( originMap, bestMuMap ); // all cells from laser pose to measurement (or, if scan matching was successful, to best hit cell)
	size_t hitIdx = line.size() - 1;

	// Cells behind measurement (not considered in python code, but otherwise the likelihood forms a sawtooth curve)
	line.extend( bestMuMap + (direction * 2).template cast<map_t>() );

	assert( line.size() <= 1000 );

	BeliefFunction	cellBeliefs[1000];
	world_t			distances[1000];
	int				j = 0;

	for( size_t i = 0; i < line.size(); i++ ) {
		const BeliefCell<N> &cell = map.cell( line[i] );
		cellBeliefs[i] = cell.mass().normalized();

		/*
		// avoid high likelihood in Theta areas (ugly hack!)
		if( cell.visits() == 0 )
			continue;
		*/

		if( i == hitIdx && cell.hits() )
			distances[j] = (sensorOrigin - cell.mean()).norm();
		else
			distances[j] = (sensorOrigin - map.map2world( line[i] )).norm();

		j++;
	}

	/*
	// add "imaginary" occupied cell for max range
	BeliefFunction	maxCellBelief(0.0, 0.0, 1.0);
	distances[j] = 15.0;
	cellBeliefs[j] = maxCellBelief;

	double lh = beliefLikelihood( dist, j + 1, cellBeliefs, distances );
	*/

	double lh = beliefLikelihood( dist, j, cellBeliefs, distances );

	if( lh > 0.0 )
		return log( lh );
	else
		return log( 1e-6 );
}



template<int N>
double
SensorModel< BeliefCell<N> >::beliefLikelihood( world_t dist, int numCells, const BeliefFunction *cellBeliefs, const world_t *distances, int i ) const {
	// according to Reineking 2014, Figure 4.4

	if( i < 0 )
		return beliefLikelihood( dist, numCells, cellBeliefs, distances, 0 ) * (1 - m_beliefRandom) + m_beliefRandom; // random measurements

	if( i == numCells )
		return getPl( dist - m_maxRange );	// virtual occupied cell

	double pl_o, pl_e, pl_Theta;

	// occupied
	pl_o = getPl( dist - distances[i] );

	// empty
	if( distances[i] > dist && pl_o < 1e-6 )
		pl_e = 0.0; // break off recursion if values become too small
	else
		pl_e = beliefLikelihood( dist, numCells, cellBeliefs, distances, i+1 );

	// Theta
	pl_Theta = pl_o + pl_e - pl_o*pl_e; 	// disjunctive rule

	const BeliefFunction &m = cellBeliefs[i];

	return m.o * pl_o + m.e * pl_e + m.Theta * pl_Theta;
}



template<int N>
double
SensorModel< BeliefCell<N> >::sigmaSqr() const {
	return m_beliefSigmaForwardSqr;
}


template<int N>
double
SensorModel< BeliefCell<N> >::nullLogLikelihood() const {
	return -0.5 / (2.0 * m_beliefSigmaForwardSqr);
}


template<int N>
bool
SensorModel< BeliefCell<N> >::excludeNoHit() const {
	return m_beliefExcludeNoHit;
}


template<int N>
int
SensorModel< BeliefCell<N> >::cellsBehind() const {
	return m_beliefCellsBehind;
}



template<int N>
void
SensorModel< BeliefCell<N> >::computeBeliefLUT() {
	/*
	// Old calculation (before IJAR 2015)
	double 	L 		= 1.0,
			L_i 	= 1.0,
			eta		= 1.0;
	double 	m, bel, pl, eta_e;
	*/

	// /*
	// New calculation (since IJAR 2015)
	double 	eta 	= 0.0,
			phi_c_i	= 0.0;
	double	m, bel, pl, eta_e, halfCellDist, sigmaDist;
	// */


	BeliefFunction *bf;

	// FIXME: Lookup table not correct, because iteration is done over more values than cells because of discretization

	m_beliefDistScale 	= 1000.0;						// default scaling factor
	/*
	l_wrn( "XXX" );
	m_beliefDistScale = 1.0 / 0.05;	// XXX
	*/

	world_t	maxDist		= m_beliefSigmaInverse * 6;	// Calculate the values up to 6 sigma. Before and after this point, the values are aprox. equal to belief[Before]Zero
	size_t	maxDistIdx 	= maxDist * m_beliefDistScale;

	if( maxDistIdx > MAX_SIZE ) {
		maxDistIdx 			= MAX_SIZE;
		m_beliefDistScale	= maxDistIdx / maxDist;
		l_wrn( "Distance scale for belief LUT was reduced to " << m_beliefDistScale << " in order to limit LUT size to " << MAX_SIZE << " entries." );
	}

	halfCellDist	= m_beliefDistScale * Config::getDouble( "MAP_DELTA", 0.05 ) * 0.5;
	sigmaDist		= m_beliefDistScale * m_beliefSigmaInverse;

	l_dbg( "Belief LUT size " << maxDistIdx << ", distance scale " << m_beliefDistScale << "." );

	size_t	numDist 	= 2 * maxDistIdx,		// in front of and behind the hit cell
			hitIdx		= maxDistIdx;			// hit cell is in the middle

	assert( maxDistIdx > 0 );

	m_beliefLUT.resize( maxDistIdx + 1 );		// zero is saved twice
	m_beliefBehindLUT.resize( maxDistIdx );


	/*
	// Old calculation (before IJAR 2015)
	std::vector<double>	pl_z( numDist );
	// belief calculation part 1 (according to Reineking 2014, Figure 4.6 first for-loop)
	for( size_t i = 0; i < numDist; i++ ) {
		world_t dist = ( i <= hitIdx ? hitIdx - i : i - hitIdx ) / m_beliefDistScale;	// actual distance to hit cell

		// pl_z[i] = NormalDistribution::pl( dist, m_beliefSigmaInverseSqr );				// pl[x_t, C=i, \not r](z_t) (a normalized Gaussian)

		pl_z[i] = NormalDistribution::cdfStddev( dist + 2*m_beliefSigmaInverse, m_beliefSigmaInverse )
					- NormalDistribution::cdfStddev( dist - 2*m_beliefSigmaInverse, m_beliefSigmaInverse );

		L *= (1.0 - pl_z[i]); 															// \prod_i=0^M (1 - pl_{z;i})
	}
	eta = 1.0 / (1.0 - L);																// normalization for GBT
	*/

	//std::cerr << "\n\n\n\n";

	// /*
	// New calculation (since IJAR 2015) (TODO: Ref to journal paper equation)
	std::vector<double> pdf( numDist );
	for( size_t i = 0; i < numDist; i++ ) {
		world_t dist = ( i <= hitIdx ? hitIdx - i : i - hitIdx ) / m_beliefDistScale;	// actual distance to hit cell
		pdf[i] = NormalDistribution::pdf( dist, m_beliefSigmaInverseSqr );				// phi( z_t, mu_c, sigma_z^2 )
		eta += pdf[i];
		//std::cerr << dist << "    " << pdf[i] << std::endl;
	}
	eta = 1.0 / eta;
	// */


	//std::cerr << 1.0 / eta << std::endl;


	// iterate over all distances
	for( size_t i = 0; i < numDist; i++ ) {
		// calc index in LUT
		if( i <= hitIdx ) {
			// in front of hit cell
			bf = &m_beliefLUT[hitIdx - i];
		} else {
			// behind hit cell
			bf = &m_beliefBehindLUT[i - hitIdx];
		}

		/*
		// Old calculation (before IJAR 2015)
		// belief calculation part 2 (according to Reineking 2014, Figure 4.6 second for-loop)
		L_i 	*= 	(1.0 - pl_z[i]);								// \prod_j=0^i (1 - pl_{z;j})
		bel		= 	eta * (L_i - L);								// bel[x_t, z_t, \not r](C=C_i)
		pl		=	eta * pl_z[i];									// pl[x_t, z_t, \not r](C=i)
		eta_e	=	1.0 / ( 1.0 - m_beliefOccPrior * (1.0 - pl) );	// normalization for e_i

		if( pl_z[i] == 1.0 ) {	// we have to prevent division by zero (compare sensor_models_laser.py -> inverse_sensor_model)
			if( i > 1 && pl_z[i-1] == 1.0 ) {
				m = 0.0;	 	// the resulting product might be 0 anyways -> check neighboring cell
			} else {
				double L_new = 1.0;
				for( int j = 0; j < numDist; j++ ) {
					if( j != i )
						L_new *= (1.0 - pl_z[j]);
				}
				m = eta * pl_z[i] * L_new;
			}
		} else {
			m =	eta * pl_z[i] * L / (1.0 - pl_z[i]);				// m[x_t, z_t, \not r](C=i)
		}

		//l_dbg( pl_z[d] << " " << L_i << " "<< m << " "<< bel<< " "<< pl << " "<< eta_e );

		bf->o		= (1 - m_beliefRandom) * eta_e * (m_beliefOccPrior * pl + (1 - m_beliefOccPrior) * m);
		bf->e		= (1 - m_beliefRandom) * eta_e * (1 - m_beliefOccPrior) * bel;
		bf->Theta 	= 1 - bf->o - bf->e;

		*/

		// /*
		// New calculation (since IJAR 2015) (TODO: Ref to journal paper equation)

		//phi_c_i	+= pdf[i];

		// Cumulative calulation of sum_{c in compl C_i) phi_c
		phi_c_i = 0;
		for( size_t j = 0; (j <= i + m_beliefSigmaInverseFactor*sigmaDist + halfCellDist) && (j < numDist); j++ )
			phi_c_i += pdf[j];

		//m		= pdf[i] * eta;

		// m(C={i}|x_t, z_t, not r)
		m = 0;
		for( size_t j = i >= halfCellDist ? i - halfCellDist : 0; (j <= i + halfCellDist) && (j < numDist); j++ ) // take cell size into account
			m += pdf[j];
		m 		*= eta;
		pl		= m;					// pl(C={i}|x_t, z_t, not r), actual the same as m for this special case
		bel		= 1 - phi_c_i * eta;	// bel(C_i|x_t, z_t, not r)

		eta_e	= 1.0 / (1 - m_beliefOccPrior * (1 - pl));

		bf->o		= (1 - m_beliefRandom) * eta_e * (m_beliefOccPrior * pl + (1 - m_beliefOccPrior) * m);
		bf->e		= (1 - m_beliefRandom) * eta_e * (1 - m_beliefOccPrior) * bel;
		bf->Theta 	= 1 - bf->o - bf->e;
		// */
	}

	// set zero dist in behindLUT as well
	m_beliefBehindLUT[0] = m_beliefLUT[0];

	m_beliefZero.set( m_beliefRandom, 0.0, 1 - m_beliefRandom );	// random on Theta, rest on empty
	m_beliefBehindZero.set( 1.0, 0.0, 0.0 ); 						// only mass on Theta
}


template<int N>
const BeliefFunction&
SensorModel< BeliefCell<N> >::getBelief( world_t dist, bool behind ) const {
	/* TODO: How to use dist^2 for the index in order to save computation time in euclidian
	 * distance calculation? Using dist^2 directly reduces the resolution of the LUT
	 * in the vicinity of the hit cell, while the resolution far away is quadraticely higher.
	 * Remember to change the distance calculation in computeLUT().
	 */
	size_t	distIdx = fabs( dist )*m_beliefDistScale + 0.5;	// 0.5 is to round the value correctly

	//l_dbg( distIdx );

	if( behind ) {
		if( distIdx >= m_beliefBehindLUT.size() )
			return m_beliefBehindZero;
		else
			return m_beliefBehindLUT[distIdx];
	} else {
		if( distIdx >= m_beliefLUT.size() )
			return m_beliefZero;
		else
			return m_beliefLUT[distIdx];
	}
}


template<int N>
void
SensorModel< BeliefCell<N> >::computePlLUT() {
	m_plDistScale = 1000.0;							// default scaling factor

	world_t	maxDist		= m_beliefSigmaForward * 6;	// Calculate the values up to 6 sigma. Before and after this point, the values are aprox. equal to zero
	size_t	maxDistIdx 	= maxDist * m_plDistScale;

	if( maxDistIdx > MAX_SIZE ) {
		maxDistIdx 		= MAX_SIZE;
		m_plDistScale 	= maxDistIdx / maxDist;
		l_wrn( "Distance scale for plausibility LUT was reduced to " << m_plDistScale << " in order to limit LUT size to " << MAX_SIZE << " entries." );
	}

	l_dbg( "Plausibility LUT size " << maxDistIdx << ", distance scale " << m_plDistScale << "." );

	m_plLUT.resize( maxDistIdx );

	for( size_t i = 0; i < maxDistIdx; i++ ) {
		// Old calculation (before IJAR 2015)
		//m_plLUT[i] = NormalDistribution::pl( i / m_plDistScale, m_beliefSigmaForwardSqr );
		//continue;

		// New calculation (since IJAR 2015)
		double dist = i / m_plDistScale;

		// CDF for an interval from z - 2*sigma to z + factor*sigma (TODO: Ref to journal paper equation)
		m_plLUT[i] = NormalDistribution::cdf( dist + m_beliefSigmaForwardFactor*m_beliefSigmaForward, m_beliefSigmaForwardSqr )
					- NormalDistribution::cdf( dist - m_beliefSigmaForwardFactor*m_beliefSigmaForward, m_beliefSigmaForwardSqr );
	}
}


template<int N>
double
SensorModel< BeliefCell<N> >::getPl( world_t dist  ) const {
	size_t	distIdx = fabs( dist )*m_plDistScale + 0.5;	// 0.5 is to round the value correctly

	if( distIdx >= m_plLUT.size() )
		return 0.0;
	else
		return m_plLUT[distIdx];
}


} /* namespace efs */
