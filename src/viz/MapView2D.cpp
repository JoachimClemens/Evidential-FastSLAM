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

#include <QMouseEvent>
#include <QKeyEvent>

#include "efslam/viz/MapView2D.h"
#include "efslam/utils/Config.h"

namespace efs {

MapView2DBase::MapView2DBase( QWidget * parent,	Qt::WindowFlags f ) :
		QWidget(parent, f),
		m_timer( 0 ),
		m_pixmap( new QPixmap( 500, 500 ) ),
		m_dragging( false ),
		m_showBestPath( true ),
		m_showPaths( false ),
		m_normalize( false ),
		m_showCovariances( false ),
		m_writeFile( 0 ),
		m_count( 0 ),
		m_mapScale( 10.0 ),
		m_viewCenter( Point2w::Zero() ),
		m_bestParticlePos( Point2w::Zero() )
{
	setAttribute( Qt::WA_NoBackground );
	setMinimumSize( 500, 500 );
	m_pixmap->fill( Qt::white );

	m_outputDir = Config::get( "OUTPUT_DIR", "." );
}


MapView2DBase::~MapView2DBase() {
	delete m_pixmap;
}


void
MapView2DBase::timerEvent( QTimerEvent *te ) {
	if( te->timerId() != m_timer )
		return;

	this->pollSlam();
	update();

	if( m_writeFile && (m_count % m_writeFile) == 0 ) {
		char name[128];
		snprintf( name, 127, "%s/map_%05d.png", m_outputDir.c_str(), (int) m_count / m_writeFile);
		QImage image = m_pixmap->toImage();
		image.save( name ,"PNG" );
	}
	m_count++;
}


void
MapView2DBase::paintEvent( QPaintEvent *paintevent ) {
	QPainter p(this);
	p.drawPixmap( QPoint(0, 0), *m_pixmap);
}


void
MapView2DBase::mousePressEvent(QMouseEvent *event) {
	if( event->button() == Qt::LeftButton ) {
		m_dragging = true;
		m_draggingPos = event->pos();
	}
}


void
MapView2DBase::mouseMoveEvent(QMouseEvent *event) {
	if( m_dragging ) {
		QPoint delta = event->pos() - m_draggingPos;
		m_draggingPos = event->pos();
		m_viewCenter[0] -= delta.x() / m_mapScale;
		m_viewCenter[1] += delta.y() / m_mapScale;
		update();
	}
}


void
MapView2DBase::mouseReleaseEvent(QMouseEvent *event) {
	if( event->button() == Qt::LeftButton ) {
		m_dragging = false;
	}
}


void
MapView2DBase::keyPressEvent(QKeyEvent* e) {
	switch( e->key() ) {
		case Qt::Key_B:
			m_showBestPath = !m_showBestPath;
			break;
		case Qt::Key_P:
			m_showPaths = !m_showPaths;
			break;
		case Qt::Key_Plus:
			m_mapScale *= 1.25;
			break;
		case Qt::Key_Minus:
			m_mapScale /= 1.25;
			break;
		case Qt::Key_C:
			m_viewCenter = m_bestParticlePos;
			break;
		case Qt::Key_N:
			m_normalize = !m_normalize;
			break;
		case Qt::Key_W:
			m_writeFile = !m_writeFile;
			break;
		case Qt::Key_V:
			m_showCovariances = !m_showCovariances;
			break;
		default:
			break;
	}
}


void
MapView2DBase::resizeEvent( QResizeEvent *sizeev ) {
	if( !m_pixmap )
		return;
	delete m_pixmap;
	m_pixmap = new QPixmap( sizeev->size() );
	QWidget::resizeEvent( sizeev );
}


void
MapView2DBase::start( int interval ) {
	m_timer = startTimer( interval );
}


} /* namespace efs */
