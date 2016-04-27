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

#ifndef EFS_NEFFVIEW_H_
#define EFS_NEFFVIEW_H_

#include <deque>
#include <functional>

#include <QWidget>
#include <QPixmap>

namespace efs {

class NeffView : public QWidget {
Q_OBJECT

public:
	using	NeffFunction			= std::function<double()>;
	using 	DequeDouble				= std::deque<double>;
	using	NumMeasurementsFunction	= std::function<uint64_t()>;

			NeffView( QWidget *parent = nullptr, Qt::WindowFlags f = 0 );
	virtual ~NeffView();

	void	start( int drawInterval, int pollInterval = 10 );
	void	setNeffFunction( NeffFunction f )	{ m_getNeff = f; }
	void	setNumMeasurementsFunction( NumMeasurementsFunction f ) { m_numMeasurements = f; }

	void	setYReference( double y )			{ m_reference = y; m_useYReference = true; }
	void	disableYReference()					{ m_useYReference = false; }

	void	setTitle( const QString &t )		{ m_title = t; }
	void	setRange( double min, double max )	{ m_min = min; m_max = max; }
	void	setNormalization( double n )		{ m_normalization = n; }

protected:
	void	timerEvent( QTimerEvent *re );
	void	keyPressEvent( QKeyEvent *e );
	void	resizeEvent( QResizeEvent *sizeev );
	void	paintEvent( QPaintEvent * );

	bool					m_autoscale;
	int						m_timer,
							m_count,
							m_drawInterval;
	QPixmap					*m_pixmap;
	NeffFunction			m_getNeff;
	NumMeasurementsFunction	m_numMeasurements;
	DequeDouble				m_values;
	double					m_min,
							m_max,
							m_normalization,
							m_reference;
	bool					m_useYReference;
	QString 				m_title;
	uint64_t				m_lastMeasurements;
};

} /* namespace efs */

#endif /* EFS_NEFFVIEW_H_ */
