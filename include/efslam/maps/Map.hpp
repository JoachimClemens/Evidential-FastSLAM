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

namespace efs {


template<class Cell, class Storage>
Map<Cell, Storage>::Map( const MapPoint &size, world_t delta ) :
	m_storage( size )
{
	initMembers( delta );
}


template<class Cell, class Storage>
Map<Cell, Storage>::Map( const WorldPoint &center, const WorldPoint &size, world_t delta ) :
	m_storage( (size / delta + WorldPoint::Ones()*0.9999).template cast<map_t>() ) // round value up
{
	initMembers( delta, &center );
}


template<class Cell, class Storage>
Map<Cell, Storage>::Map( const WorldPoint &center, const WorldPoint &min, const WorldPoint &max, world_t delta ) :
	m_storage( ((max - min) / delta + WorldPoint::Ones()*0.9999).template cast<map_t>() ) // round value up
{
	WorldPoint newCenter = center - min;
	initMembers( delta, &newCenter );
}


template<class Cell, class Storage>
void
Map<Cell, Storage>::clear()
{
	m_storage.clear();
}


template<class Cell, class Storage>
void
Map<Cell, Storage>::initMembers( world_t delta, const WorldPoint *center ) {
	// the actual size of the storage may differ from the params given to the constructor
	m_mapSize		= m_storage.size();
	//m_mapSizeHalf	= m_mapSize / 2;	// Attention: m_mapSize * 0.5 produce zero!
	m_worldSize		= m_mapSize.template cast<world_t>() * delta;
	m_delta			= delta;
	if( center )
		m_center = *center;
	else
		m_center = m_worldSize / 2;
}


template<class Cell, class Storage>
Map<Cell, Storage>::~Map() {
	// Nothing to do here?
}


template<class Cell, class Storage>
typename Map<Cell, Storage>::MapPoint
Map<Cell, Storage>::world2map( const WorldPoint &p ) const {
	/*
	// Calculation from GMapping: Produces wired/wrong results
	MapPoint res;

	for( int i = 0; i < N; i++ )
		res[i] = round( (p[i] - m_center[i]) / m_delta ) + m_mapSizeHalf[i];

	return res;
	*/

	// Calculation with Eigen
	//return ((p + m_center) / m_delta + sm_point5).template cast<int>();

	// Same calculation, but should be faster
	MapPoint res;

	for( int i = 0; i < Dimension; i++ )
		res[i] = ((p[i] + m_center[i]) / m_delta + 0.5);

	return res;
}


template<class Cell, class Storage>
typename Map<Cell, Storage>::WorldPoint
Map<Cell, Storage>::map2world( const MapPoint &p ) const {
	// Calculation from GMapping: Produces wired/wrong results
	//return (p - m_mapSizeHalf) * m_delta;

	// Calculation from ds-slam: Better, but not correct at all
	//return (p.template cast<double>() - sm_point5) * m_delta - m_center;

	// Calculation with Eigen
	//return p.template cast<double>() * m_delta - m_center;

	// Same calculation, but should be faster
	WorldPoint res;

	for( int i = 0; i < Dimension; i++ )
		res[i] = p[i] * m_delta - m_center[i];

	return res;
}


template<class Cell, class Storage>
size_t
Map<Cell, Storage>::bytes() const {
	return sizeof(*this) + m_storage.bytes();
}


template<class Cell, class Storage>
void
Map<Cell, Storage>::load( std::istream &f ) {
	bool 	headerComplete = false;
	int		lineNum = 1;

	WorldPoint 	center	= WorldPoint::Zero();
	MapPoint	size	= MapPoint::Zero();
	world_t		delta	= 0.0;

	while( f.good() ) {
		std::string line;
		std::getline( f, line );

		if( line.empty() || line[0] == '#' )
			continue;

		auto splitted = split( line, ':' );

		if( splitted.size() != 2 )
			throw std::runtime_error( to_string( "Wrong number of `:' in line " ) + to_string( lineNum ) );

		if( !headerComplete ) {
			if( splitted[0] == "cell" ) {
				if( std::stol( splitted[1] ) != Cell::staticType() )
					throw std::runtime_error( to_string( "Wrong cell type in line " + to_string( lineNum ) + ", expected " + to_string( Cell::staticType() ) + ", got " + to_string( std::stol( splitted[1] ) ) ) );
			} else if( splitted[0] == "dimension" ) {
				if( std::stol( splitted[1] ) != Dimension )
					throw std::runtime_error( to_string( "Wrong map dimension in line " + to_string( lineNum ) + ", expected " + to_string( (int) Dimension ) + ", got " + to_string( std::stol( splitted[1] ) ) ) );
			} else if( splitted[0] == "size" ) {
				auto splitted2 = split( splitted[1], ' ' );

				if( splitted2.size() != Dimension )
					throw std::runtime_error( to_string( "Wrong number of values in line " ) + to_string( lineNum ) + ", expected " + to_string( (int) Dimension ) + ", got " + to_string( splitted2.size() ) );

				for( int i = 0; i < Dimension; i++ )
					size[i] = std::stol( splitted2[i] );
			} else if( splitted[0] == "delta") {
				delta = std::stod( splitted[1] );
			} else if( splitted[0] == "center" ) {
				auto splitted2 = split( splitted[1], ' ' );

				if( splitted2.size() != Dimension )
					throw std::runtime_error( to_string( "Wrong number of values in line " ) + to_string( lineNum ) + ", expected " + to_string( (int) Dimension ) + ", got " + to_string( splitted2.size() ) );

				for( int i = 0; i < Dimension; i++ )
					center[i] = std::stod( splitted2[i] );
			} else {
				if( delta == 0 || size == MapPoint::Zero() )
					throw std::runtime_error( to_string( "Delta or size is zero" ) );

				m_storage = Storage( size );
				initMembers( delta, &center );

				headerComplete = true;
			}
		}

		if( headerComplete ) {
			auto splitted2 = split( splitted[0], ' ' );

			if( splitted2.size() != Dimension )
				throw std::runtime_error( to_string( "Wrong number of values in line " ) + to_string( lineNum ) + ", expected " + to_string( (int) Dimension ) + ", got " + to_string( splitted2.size() ) );

			try {
				MapPoint p;
				for( int i = 0; i < Dimension; i++ )
					p[i] = std::stol( splitted2[i] );

				cell( p ).fromStr( splitted[1] );
			} catch( std::runtime_error &e ) {
				throw std::runtime_error( to_string( "Failed to read cell from line " ) + to_string( lineNum ) + ": " + e.what() );
			}
		}

		lineNum++;
	}
}


template<class Cell, class Storage>
void
Map<Cell, Storage>::save( std::ostream &f ) const {
	f << "cell: " << Cell::staticType() << std::endl;
	f << "dimension: " << Dimension << std::endl;
	f << "size: " << m_mapSize.transpose() << std::endl;
	f << "delta: " << m_delta << std::endl;
	f << "center: " << m_center.transpose() << std::endl;

	for( const auto &c : cells() )
		f << c.first.transpose() << ": " << c.second.toStr() << std::endl;
}



} /* namespace efs */
