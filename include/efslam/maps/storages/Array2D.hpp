/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping HArray2D class
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


namespace efs {

template<class Cell, const bool debug>
Array2D<Cell, debug>::Array2D( const Point2m &size ) {
//	assert(xsize>0);
//	assert(ysize>0);
	m_size = size;
	if (m_size[0] > 0 && m_size[1] > 0) {
		m_cells = new Cell*[m_size[0]];
		for (int i = 0; i < m_size[0]; i++)
			m_cells[i] = new Cell[m_size[1]];
	} else {
		m_size[0] = m_size[1] = 0;
		m_cells = 0;
	}
	if (debug) {
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_size[0]= " << m_size[0] << std::endl;
		std::cerr << "m_size[1]= " << m_size[1] << std::endl;
	}
}


template<class Cell, const bool debug>
Array2D<Cell, debug>::Array2D( map_t xsize, map_t ysize ) {
//	assert(xsize>0);
//	assert(ysize>0);
	m_size[0] = xsize;
	m_size[1] = ysize;
	if (m_size[0] > 0 && m_size[1] > 0) {
		m_cells = new Cell*[m_size[0]];
		for (int i = 0; i < m_size[0]; i++)
			m_cells[i] = new Cell[m_size[1]];
	} else {
		m_size[0] = m_size[1] = 0;
		m_cells = 0;
	}
	if (debug) {
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_size[0]= " << m_size[0] << std::endl;
		std::cerr << "m_size[1]= " << m_size[1] << std::endl;
	}
}


template<class Cell, const bool debug>
Array2D<Cell, debug> & Array2D<Cell, debug>::operator=( const Array2D<Cell, debug> &g ) {
	if (debug || m_size[0] != g.m_size[0] || m_size[1] != g.m_size[1]) {
		for (map_t i = 0; i < m_size[0]; i++)
			delete[] m_cells[i];
		delete[] m_cells;
		m_size[0] = g.m_size[0];
		m_size[1] = g.m_size[1];
		m_cells = new Cell*[m_size[0]];
		for (map_t i = 0; i < m_size[0]; i++)
			m_cells[i] = new Cell[m_size[1]];
	}
	for (map_t x = 0; x < m_size[0]; x++)
		for (map_t y = 0; y < m_size[1]; y++)
			m_cells[x][y] = g.m_cells[x][y];

	if (debug) {
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_size[0]= " << m_size[0] << std::endl;
		std::cerr << "m_size[1]= " << m_size[1] << std::endl;
	}
	return *this;
}


template<class Cell, const bool debug>
Array2D<Cell, debug> & Array2D<Cell, debug>::operator=( Array2D<Cell, debug> &&g ) {
	if( this != &g ) {
		for (map_t i = 0; i < m_size[0]; i++)
			delete[] m_cells[i];
		delete[] m_cells;
		m_size	 	= g.m_size;
		m_cells		= g.m_cells;
		g.m_cells	= 0;
	}
	return *this;
}


template<class Cell, const bool debug>
Array2D<Cell, debug>::Array2D( const Array2D<Cell, debug> &g ) {
	m_size = g.m_size;
	m_cells = new Cell*[m_size[0]];
	for (map_t x = 0; x < m_size[0]; x++) {
		m_cells[x] = new Cell[m_size[1]];
		for (map_t y = 0; y < m_size[1]; y++)
			m_cells[x][y] = g.m_cells[x][y];
	}
	if (debug) {
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_size[0]= " << m_size[0] << std::endl;
		std::cerr << "m_size[1]= " << m_size[1] << std::endl;
	}
}


template<class Cell, const bool debug>
Array2D<Cell, debug>::Array2D( Array2D<Cell, debug> &&g ) {
	m_size	 	= g.m_size;
	m_cells		= g.m_cells;
	g.m_cells	= 0;
}


template<class Cell, const bool debug>
Array2D<Cell, debug>::~Array2D() {
	if (debug) {
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_size[0]= " << m_size[0] << std::endl;
		std::cerr << "m_size[1]= " << m_size[1] << std::endl;
	}
	if( m_cells ) {
		for (map_t i = 0; i < m_size[0]; i++) {
			if( m_cells[i] ) {
				delete[] m_cells[i];
				m_cells[i] = 0;
			}
		}
		delete[] m_cells;
		m_cells = 0;
	}
}


template<class Cell, const bool debug>
void
Array2D<Cell, debug>::clear() {
	/*
	if (debug) {
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_size[0]= " << m_size[0] << std::endl;
		std::cerr << "m_size[1]= " << m_size[1] << std::endl;
	}
	for (map_t i = 0; i < m_size[0]; i++) {
		delete[] m_cells[i];
		m_cells[i] = 0;
	}
	delete[] m_cells;
	m_cells = 0;
	m_size[0] = 0;
	m_size[1] = 0;
	*/
	for (map_t x = 0; x < m_size[0]; x++)
		for (map_t y = 0; y < m_size[1]; y++)
			m_cells[x][y] = Cell( 0 );
}


template<class Cell, const bool debug>
void
Array2D<Cell, debug>::resize( map_t xmin, map_t ymin, map_t xmax, map_t ymax ) {
	map_t xsize = xmax - xmin;
	map_t ysize = ymax - ymin;
	Cell ** newcells = new Cell *[xsize];
	for (int x = 0; x < xsize; x++) {
		newcells[x] = new Cell[ysize];
	}
	map_t dx = xmin < 0 ? 0 : xmin;
	map_t dy = ymin < 0 ? 0 : ymin;
	map_t Dx = xmax < this->m_size[0] ? xmax : this->m_size[0];
	map_t Dy = ymax < this->m_size[1] ? ymax : this->m_size[1];
	for (map_t x = dx; x < Dx; x++) {
		for (map_t y = dy; y < Dy; y++) {
			newcells[x - xmin][y - ymin] = this->m_cells[x][y];
		}
		delete[] this->m_cells[x];
	}
	delete[] this->m_cells;
	this->m_cells = newcells;
	this->m_size[0] = xsize;
	this->m_size[1] = ysize;
}


template<class Cell, const bool debug>
bool
Array2D<Cell, debug>::isInside( map_t x, map_t y ) const {
	return x >= 0 && y >= 0 && x < m_size[0] && y < m_size[1];
}


template<class Cell, const bool debug>
const Cell&
Array2D<Cell, debug>::cell( map_t x, map_t y ) const {
	assert(isInside(x, y));
	return m_cells[x][y];
}


template<class Cell, const bool debug>
Cell&
Array2D<Cell, debug>::cell( map_t x, map_t y ) {
	assert(isInside(x, y));
	return m_cells[x][y];
}


template<class Cell, const bool debug>
template<typename CellOut>
typename Array2D<Cell, debug>::template CellList<CellOut>
Array2D<Cell, debug>::cells( const Point2m& offset ) const {
	CellList<CellOut> res;

	for( map_t x = 0; x < m_size[0]; x++ )
		for( map_t y = 0; y < m_size[1]; y++ )
			res.push_back( std::pair<Point2m, const CellOut &>( offset + Point2m( x, y ), m_cells[x][y] ) );

	return res;
}


} /* namespace efs  */
