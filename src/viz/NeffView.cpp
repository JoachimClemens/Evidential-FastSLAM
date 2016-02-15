/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2013-2016, Joachim Clemens, Thomas Reineking, Tobias Kluth
 *  All rights reserved.
 *
 *  This file is partially based on the GMapping QGraphPainter class
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

#include <QTimerEvent>
#include <QResizeEvent>
#include <QKeyEvent>
#include <QPainter>

#include "efslam/viz/NeffView.h"

namespace efs {

NeffView::NeffView(  QWidget *parent, Qt::WFlags f  ) :
	QWidget( parent, f ),
	m_autoscale( false ),
	m_timer( 0 ),
	m_count( 0 ),
	m_drawInterval( 0 ),
	m_pixmap( new QPixmap(size()) ),
	m_min( 0 ),
	m_max( 1 ),
	m_normalization( 1 ),
	m_reference( 0 ),
	m_useYReference( false ),
	m_lastMeasurements( 0 )
{
	m_pixmap->fill(Qt::white);
}


NeffView::~NeffView() {
	delete m_pixmap;
}


void
NeffView::keyPressEvent( QKeyEvent *e ) {
	switch( e->key() ) {
		case Qt::Key_A:
			m_autoscale = !m_autoscale;
			break;
		default:
			break;
	}
}


void
NeffView::resizeEvent( QResizeEvent * sizeev ) {
	delete m_pixmap;
	m_pixmap = new QPixmap( sizeev->size() );
	QWidget::resizeEvent( sizeev );
}


void
NeffView::timerEvent( QTimerEvent *te ) {
	if( te->timerId() != m_timer || !m_getNeff || !m_numMeasurements )
		return;

	uint64_t measurements = m_numMeasurements();
	if( measurements != m_lastMeasurements ) {
		m_values.push_back( m_getNeff() / m_normalization );
		m_lastMeasurements = measurements;
	}

	if( (m_count % m_drawInterval) == 0 )
		update();

	m_count++;
}


void
NeffView::start( int drawInterval, int pollInterval ) {
	m_drawInterval 	= drawInterval / pollInterval;
	m_count 		= 0;
	m_timer 		= startTimer( pollInterval );
}


void
NeffView::paintEvent(QPaintEvent *) {
	m_pixmap->fill(Qt::white);
	QPainter painter(m_pixmap);
	double	min = std::numeric_limits<double>::max(),
			max = -std::numeric_limits<double>::max();
	if( m_autoscale ) {
		for( unsigned int i = 0; i < (unsigned int) width() && i < m_values.size(); i++ ) {
			min = min < m_values[i] ? min : m_values[i];
			max = max > m_values[i] ? max : m_values[i];
		}
	} else {
		min = m_min;
		max = m_max;
	}

	painter.setPen( Qt::black );
	painter.drawRect( 0, 0, width(), height() );
	const int	boundary 	= 2;
	int 		xoffset		= 40;
	double scale = ((double) height() - 2 * boundary - 2) / (max - min);

	if( m_useYReference ) {
		painter.setPen( Qt::green );
		painter.drawLine( 	xoffset + boundary / 2,
							height() - (int) (scale * (m_reference - min)),
							width() - boundary / 2,
							height() - (int) (scale * (m_reference - min)) );
	}

	painter.setPen( Qt::blue );

	unsigned int start = 0;
	if( m_values.size() > (unsigned int) width() - 2 * boundary - xoffset )
		start = m_values.size() - width() + 2 * boundary + xoffset;

	int oldv = 0;
	if( (unsigned int) width() - 2 * boundary - xoffset > 1 && m_values.size() > 1 )
		oldv = (int) (scale * (m_values[1 + start] - min)) + boundary;

	for( unsigned int i = 1; i < (unsigned int) width() - 2 * boundary - xoffset && i < m_values.size(); i++ ) {
		int v = (int) (scale * (m_values[i + start] - min)) + boundary;
		painter.drawLine(i - 1 + boundary + xoffset, height() - boundary - oldv, xoffset + i + boundary, height() - boundary - v);
		oldv = v;
	}

	painter.setPen( Qt::black );
	painter.drawText( 3, height() / 2, m_title );
	QFont sansFont( "Helvetica [Cronyx]", 6 );
	painter.setFont( sansFont );

	QPainter p( this );
	p.drawPixmap( QPoint(0, 0), *m_pixmap );
}

} /* namespace efs */
