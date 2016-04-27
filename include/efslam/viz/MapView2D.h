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

#ifndef EFS_MAPVIEW2D_H_
#define EFS_MAPVIEW2D_H_

#include <QWidget>
#include <QPixmap>
#include <QPainter>
#include <QImage>

#include "efslam/utils/Point.h"

namespace efs {

// Workaround to use template based QObjects: http://doc.qt.digia.com/qq/qq15-academic.html


class MapView2DBase : public QWidget {
Q_OBJECT

public:
					MapView2DBase( QWidget *parent = nullptr, Qt::WindowFlags f = 0 );
	virtual			~MapView2DBase();

public slots:
	virtual void	timerEvent( QTimerEvent *te );
	virtual void 	resizeEvent( QResizeEvent *sizeev );

	virtual	void	start( int interval = 1000 );

protected:
	virtual	void	pollSlam() = 0;

	inline	Point2w	pic2world( int x, int y ) {
		return m_viewCenter + Point2w( (world_t) ((x - m_pixmap->width() / 2) / m_mapScale), (world_t) ((m_pixmap->height() / 2 - y) / m_mapScale) );
	}
	inline	Point2m	world2pic( world_t x, world_t y ) {
		return Point2m( (map_t) ((x - m_viewCenter[0]) * m_mapScale) + m_pixmap->width() / 2, (map_t) ((m_viewCenter[1] - y) * m_mapScale) + m_pixmap->height() / 2);
	}
	inline	Point2m world2pic( const Point2w &p ) {
		return world2pic( (world_t) p[0], (world_t) p[1] );
	}

	virtual void 	paintEvent( QPaintEvent *paintevent );

	//mouse movement
	virtual void 	mousePressEvent( QMouseEvent* );
	virtual void 	mouseReleaseEvent( QMouseEvent* );
	virtual void 	mouseMoveEvent( QMouseEvent* );

	virtual void 	keyPressEvent( QKeyEvent* e );

	int			m_timer;
	QPixmap		*m_pixmap;

	QPoint		m_draggingPos;
	bool 		m_dragging;

	bool		m_showBestPath,
				m_showPaths,
				m_normalize,
				m_showCovariances;
	uint32_t	m_writeFile;
	uint64_t	m_count;

	//map painting
	double 		m_mapScale;
	Point2w		m_viewCenter,
				m_bestParticlePos;

	std::string	m_outputDir;
};


template<typename SLAM>
class MapView2D : public MapView2DBase {
public:
	using	MapType				= typename SLAM::MapType;
	using	Trajectory			= typename SLAM::Trajectory;
	using	TrajectoryVector	= typename SLAM::TrajectoryVector;

					MapView2D( QWidget *parent = nullptr, Qt::WFlags f = 0 );
	virtual			~MapView2D();

			void	setSlam( const SLAM *slam )	{ m_slam = slam; }

protected:

	virtual	void 	pollSlam();

			void	drawMap( const MapType &map );
			void	drawTrajectory( const Trajectory &t, const QColor &color = Qt::yellow );
			void	drawPosVector( const std::vector<Point2w> &vec );
			void	drawCovariances( const Trajectory &t, const std::vector<Eigen::Matrix3d> &covs, const std::vector<size_t> &timePoints, const QColor &color = Qt::yellow );

	const SLAM	*m_slam;
};


} /* namespace efs */


#include "MapView2D.hpp"

#endif /* EFS_MAPVIEW2D_H_ */
