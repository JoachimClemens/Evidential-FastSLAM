/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping QMapViewer class
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

#include "efslam/maps/cells/CellBase.h"
#include "efslam/maps/cells/BayesCell.h"
#include "efslam/maps/cells/BeliefCell.h"

#include "efslam/slam/SLAMTypes.h"
#include "efslam/utils/CovarianceEllipse.h"

#include "CellColor.h"


namespace efs {

template<typename SLAM>
MapView2D<SLAM>::MapView2D( QWidget *parent, Qt::WindowFlags f ) :
	MapView2DBase( parent, f ),
	m_slam( nullptr )
{
}


template<typename SLAM>
MapView2D<SLAM>::~MapView2D() {

}


template<typename SLAM>
void
MapView2D<SLAM>::pollSlam() {
	if( !this->m_slam )
		return;

	Trajectory 						traj;
	TrajectoryVector				allTraj;
	std::vector<Eigen::Matrix3d>	covs;
	std::vector<size_t>				timePoints;


	this->m_slam->lock();
	size_t	idx 				= this->m_slam->bestParticleIdx( false );
	const	MapType	map			= this->m_slam->map( idx, false );
	this->m_bestParticlePos		= this->m_slam->pose( idx ).pos();

	if( m_showBestPath ) {
		traj = this->m_slam->trajectory( idx, false );

		if( m_showCovariances && traj.size() > 9 ) {
			timePoints.reserve( traj.size() / 3 - 2 );
			for( size_t i = 1; i < traj.size() - 1; i += 3 )
				timePoints.push_back( i );

			covs = this->m_slam->covariances( timePoints, false );
		}
	}

	if( m_showPaths )
		allTraj = this->m_slam->trajectories( false );

	std::vector<Point2w> posVec;
	posVec.reserve( this->m_slam->numParticles() );
	for( size_t i = 0; i < this->m_slam->numParticles(); i++ )
		posVec.push_back( this->m_slam->pose( i ).pos() );
	this->m_slam->unlock();

	// The order of drawing is important in order not to overdraw something
	drawMap( map );
	if( m_showPaths )
		for( const auto &t : allTraj )
			drawTrajectory( t, Qt::darkYellow );
	if( m_showBestPath ) {
		if( m_showCovariances )
			drawCovariances( traj, covs, timePoints, Qt::darkYellow );
		drawTrajectory( traj );
	}
	drawPosVector( posVec );
}



template<typename SLAM>
void
MapView2D<SLAM>::drawMap( const MapType& map ) {
	//cout << "Map received" << map.getMapSizeX() << " " << map.getMapSizeY() << endl;
	QPainter painter( this->m_pixmap );
	painter.setPen( Qt::black );

	Color color;

	// Background
	if( MapType::CellType::staticType() == CellBase::CELL_BELIEF )
		color = CellColor< BeliefCell<2> >::defaultColor();
	else
		color = CellColor< BayesCell<2> >::defaultColor();
	this->m_pixmap->fill( QColor( color.r, color.g, color.b ) );


	/*
	Point2w	wmin = Point(	pic2map( Point2m( -this->m_pixmap->width() / 2, this->m_pixmap->height() / 2) ) );
	Point2w wmax = Point(	pic2map( Point2m( this->m_pixmap->width() / 2, -this->m_pixmap->height() / 2) ) );
	Point2m	imin = map.world2map( wmin );
	Point2m	imax = map.world2map( wmax );
	*/

	/*	cout << __PRETTY_FUNCTION__ << endl;
	 cout << " viewCenter=" << viewCenter.x << "," << viewCenter.y <<   endl;
	 cout << " wmin=" << wmin.x << "," << wmin.y <<  " wmax=" << wmax.x << "," << wmax.y << endl;
	 cout << " imin=" << imin.x << "," << imin.y <<  " imax=" << imax.x << "," << imax.y << endl;
	 cout << " mapSize=" << map.getMapSizeX() << "," << map.getMapSizeY() << endl;*/

	for( int x = 0; x < this->m_pixmap->width(); x++ )
		for( int y = 0; y < this->m_pixmap->height(); y++ ) {
			Point2w p = pic2world( x, y );

			if( map.isInside( p ) ) {
				const CellBase	&cellBase = map[p];

				switch( cellBase.type() ) {
					case CellBase::CELL_BAYES:
						{
							const BayesCell<2> 	&cell	= dynamic_cast<const BayesCell<2> &>( cellBase );
							if( cell.fullness() < 0 )
								continue;
							color = CellColor< BayesCell<2> >::getColor( cell );
						}
						break;

					case CellBase::CELL_BELIEF:
						{
							const BeliefCell<2> &cell	= dynamic_cast<const BeliefCell<2> &>( cellBase );
							if( cell.fullness() < 0 )
								continue;
							color = CellColor< BeliefCell<2> >::getColor( cell, m_normalize );
						}
						break;

					default:
						continue;
				}
				painter.setPen( QColor( color.r, color.g, color.b ) );
				painter.drawPoint( x, y );
			}
		}
}


template<typename SLAM>
void
MapView2D<SLAM>::drawTrajectory( const Trajectory &t, const QColor &color ) {
	QPainter painter( this->m_pixmap );
	painter.setPen( color );

	Point2m p0		= Point2m::Zero();
	bool	first 	= true;

	for( const auto &pose : t ) {
		Point2m p1 = this->world2pic( pose.pos() );

		if( !first ) {
			painter.drawLine( (int) p0[0], (int) p0[1], (int) p1[0], (int) p1[1] );
		} else {
			first = false;
		}
		p0 = p1;
	}
}


template<typename SLAM>
void
MapView2D<SLAM>::drawPosVector( const std::vector<Point2w> &vec ) {
	QPainter painter( this->m_pixmap );
	painter.setPen( Qt::yellow );

	for( const auto &pos : vec ) {
		Point2m p1 = this->world2pic( pos );
		painter.drawPoint( (int) p1[0], (int) p1[1] );
	}
}


template<typename SLAM>
void
MapView2D<SLAM>::drawCovariances( const Trajectory &t, const std::vector<Eigen::Matrix3d> &covs, const std::vector<size_t> &timePoints, const QColor &color ) {
	QPainter painter( this->m_pixmap );
	painter.setPen( color );

	for( size_t i = 0; i < timePoints.size(); i++ ) {
		CovarianceEllipse covEllipse( covs[i].template cast<world_t>().topLeftCorner<2, 2>() );
		auto lengthsWorld 	= covEllipse.lengths();
		auto lenthsPic		= lengthsWorld * m_mapScale;
		auto pos			= this->world2pic( t[timePoints[i]].pos() );

		painter.translate( (int) pos[0], (int) pos[1] );
		painter.rotate( RAD2DEG( -covEllipse.rotation() ) );	// rotate counter clockwise
		painter.drawEllipse( -(int) lenthsPic[0] / 2.0, -(int) lenthsPic[1] / 2.0, (int) lenthsPic[0], (int) lenthsPic[1] );

		painter.resetTransform();
	}

}


} /* namespace efs */


