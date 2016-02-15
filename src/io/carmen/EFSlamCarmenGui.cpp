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

#include <QMainWindow>
#include <QVBoxLayout>
#include <QApplication>
#include <QLabel>

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>

#include "efslam/io/carmen/CarmenProcessor.h"

#include "efslam/slam/BeliefFastSLAM2D.h"
#include "efslam/slam/BayesFastSLAM2D.h"

#include "efslam/utils/Config.h"
#include "efslam/utils/Log.h"
#include "efslam/utils/Version.h"

#include "efslam/viz/MapView2D.h"
#include "efslam/viz/NeffView.h"

using namespace efs;
using namespace std;


template<typename SLAM>
class MainWindow : public QMainWindow {
public:
	MainWindow( const SLAM* slam ) {
		layout = new QVBoxLayout;
		checkFinished = []() { return false; };

		setCentralWidget(new QWidget);
		centralWidget()->setLayout(layout);

		mview = new MapView2D<SLAM>( this, 0 );
		mview->setGeometry( 0, 0, 500, 500 );
		mview->setFocusPolicy( Qt::ClickFocus );
		mview->setSlam( slam );
		//mviewer->writeToFile = 1;
		layout->addWidget( mview );

		if( SLAM::type() == SLAM_FAST ) {
			neffview = new NeffView( this );
			neffview->setFixedHeight( 100 );
			neffview->setFocusPolicy( Qt::ClickFocus );
			layout->addWidget( neffview );
			neffview->setRange( 0, 1 );
			neffview->setTitle( "Neff" );
			neffview->setNormalization( slam->numParticles() );
			neffview->setNeffFunction( std::bind( []( const SLAM *slam ) { return slam->Neff(); }, slam ) );
			neffview->setNumMeasurementsFunction( std::bind( []( const SLAM *slam ) { return slam->numMeasurements(); }, slam ) );
		} else {
			neffview = nullptr;
		}

		help = new QLabel(QString("+/- - zoom | b - show/hide best path | p - show/hide all paths | v - show/hide covariances | c - center robot | n - normalize BF | a - autoscale Neff | w - write to file"), this);
		help->setMaximumHeight(30);
		layout->addWidget(help);

		//QObject::connect( mview, SIGNAL(neffChanged(double)), gpainter, SLOT(valueAdded(double)));
		setTabOrder( mview, mview );
	}

	~MainWindow() {
		delete layout;
		delete mview;
		if( neffview )
			delete neffview;
	}

	void start( int c ) {
		mview->start( c );
		if( neffview )
			neffview->start( c );
	}

	void startWatchdog( int interval, std::function<bool()>	fun ) {
		checkFinished = fun;
		startTimer( interval );
	}

	void timerEvent( QTimerEvent *te ) {
		if( checkFinished() ) {
			l_inf( "Processing finished. Exiting..." );
			qApp->quit();
		}
	}

protected:
	QVBoxLayout			*layout;
	MapView2D<SLAM>	*mview;
	NeffView				*neffview;
	QLabel					*help;
	std::function<bool()>	checkFinished;
};


template<typename SLAM>
class BatchGui {
public:
	BatchGui( const std::string outputDir ) {
		std::string filename = Config::getString( "FILENAME" );
		fp = make_shared<ifstream>( filename );
		l_inf( "Filename: " << filename );
		if( !fp->is_open() )
			throw ios::failure( "Failed to open file `" + filename + "'" );

		filename = Config::getString( "GT_FILENAME", "" );
		if( filename != "" ) {
			gtFp = make_shared<ifstream>( filename );
			l_inf( "Ground truth filename: " << filename );
			if( !gtFp->is_open() ) {
				l_wrn( "Failed to open ground truth file `" + filename + "'" );
				gtFp = nullptr;
			}
		} else {
			l_inf( "No ground truth filename given." );
		}

		processor		= make_shared< CarmenProcessor<SLAM> >( &slam, *fp, gtFp.get() );
		mainWin 		= make_shared< MainWindow<SLAM> >( &slam );
		processorThread = thread( [this, outputDir](){
			Stopwatch timer;
			uint64_t scans = processor->processAll();
			Stopwatch::milliseconds timeMs = timer.timePast();

			l_inf( "Processed scans:      \t" << scans );
			l_inf( "Total processing time:\t" << timeMs );

			size_t	bestIdx 	= slam.bestParticleIdx();

			l_inf( "Saving map..." );
			const auto &map = slam.map( bestIdx );
			ofstream mapFile( outputDir + "/bestMap.map" );
			map.save( mapFile );
			mapFile.close();

			l_inf( "Saving best trajectory..." );
			const auto &traj = slam.trajectory( bestIdx );
			ofstream trajFile( outputDir + "/bestTrajectory.csv" );
			trajFile << "# x[m] y[m] phi[deg]" << std::endl;
			for( auto &pose : traj )
				trajFile << pose << std::endl;
			trajFile.close();

			l_inf( "Saving all trajectories..." );
			const auto &allTraj = slam.trajectories();
			ofstream allTrajFile( outputDir + "/allTrajectories.csv" );
			allTrajFile << "# [ x[m] y[m] phi[deg], ... ]" << std::endl;
			for( auto &t : allTraj ) {
				allTrajFile << "[ ";
				for( auto &pose : t )
					allTrajFile << pose << ",";
				allTrajFile << "]" << std::endl;
			}
			allTrajFile.close();

			l_inf( "Saving stats..." );
			ofstream statsFile( outputDir + "/stats.txt" );
			statsFile << "Processed scans:              \t" << scans << std::endl;
			statsFile << "Total processing time (ms):   \t" << timeMs.count() << std::endl;
			size_t size = 0;
			for( size_t i = 0; i < slam.numParticles(); i++ )
				size += slam.map( i, false ).bytes();
			statsFile << "Total size of all maps (Bytes):\t" << size << std::endl;
			statsFile.close();

			l_inf( "Saving timestamps..." );
			const auto &timestamps = processor->timestamps();
			ofstream timestampsFile( outputDir + "/timestamps.txt" );
			for( auto &ts : timestamps )
				timestampsFile << ts << std::endl;
			timestampsFile.close();

			l_inf( "Saving ground truth trajectory..." );
			const auto &gtTrajectory = processor->gtTrajectory();
			ofstream gtTrajectoryFile( outputDir + "/gtTrajectory.csv" );
			trajFile << "# x[m] y[m] phi[deg]" << std::endl;
			for( auto &pose : gtTrajectory )
				gtTrajectoryFile << pose << std::endl;
			gtTrajectoryFile.close();

			l_inf( "Saving pose error..." );
			const auto &poseError = processor->poseError();
			ofstream poseErrorFile( outputDir + "/poseError.txt" );
			for( auto &e : poseError )
				poseErrorFile << e << std::endl;
			poseErrorFile.close();

			l_inf( "Done." );
		} );

		if( !Config::get( "NO_GUI", 0 ) ) {
			mainWin->show();
			mainWin->start( 1000 );
		} else {
			mainWin->startWatchdog( 1000, [this]() { return processorThread.joinable(); } );
		}
	}

	~BatchGui() {
		processorThread.join();
	}


private:
	SLAM		slam;
	thread		processorThread;
	shared_ptr< ifstream >					fp,
											gtFp;
	shared_ptr< CarmenProcessor<SLAM> >		processor;
	shared_ptr< MainWindow<SLAM> >	mainWin;
};


int
main( int argc, char *argv[] ) {
	for( auto &line : Version::infoLines() )
		l_inf( line );

	if( argc == 2 && std::string( "--help" ) == argv[1] ) {
		cerr << "Usage: " << argv[0] << " [<configfile> | <params...>]" << endl;
		return -1;
	}

	shared_ptr< BatchGui<BeliefFastSLAM2D>	>	beliefFastGui;
	shared_ptr< BatchGui<BayesFastSLAM2D> >	bayesFastGui;

	QApplication app( argc, argv );
	setlocale( LC_NUMERIC, "C" );	// ensure, that '.' is the decimal separator for stof etc.

	try {
		// Init config
		l_inf( "Loading config defaults..." );
		Config::loadDefaults();
		if( argc == 2 ) {
			l_inf( "Loading config file `" << argv[1] << "'..." );
			Config::loadFile( argv[1], false );
		} else {
			l_inf( "Parsing command line arguments to config..." );
			Config::loadParams( argc, argv, false );
		}
		l_inf( "Creating output dir..." );
		Config::createOutputDir( "carmen_efslam" );

		l_inf( "Saving config to output dir..." );
		std::string outputDir = Config::get( "OUTPUT_DIR", "." );
		l_inf( "Output dir: " << outputDir );
		Config::saveFile( outputDir + "/config.cfg" );
		l_inf( "Writing version info to output dir..." );
		std::ofstream versionFile( outputDir + "/version.txt" );
		versionFile << "Version:      " << Version::version() << std::endl;
		versionFile << "Commit hash:  " << Version::commitHash() << std::endl;
		versionFile << "Commit date:  " << Version::commitDate() << std::endl;
		versionFile << "Build date:   " << Version::buildDate() << std::endl;
		versionFile.close();

		std::string mapType		= Config::get( "MAP_TYPE",	"belief" 	);
		mapType 	= to_lower( mapType );

		l_inf( "Map type:  " << mapType );
		if( mapType == "belief" ) {
			beliefFastGui = make_shared< BatchGui<BeliefFastSLAM2D> >( outputDir );
		} else if( mapType == "bayes" ) {
			bayesFastGui = make_shared< BatchGui<BayesFastSLAM2D> >( outputDir );
		} else {
			throw invalid_argument( "Invalid map type: " + mapType );
		}

	} catch( exception &e ) {
		l_err( "Program startup failed: " << e.what() );
		return -1;
	}

	return app.exec();
}

