/*
 * Software License Agreement (BSD License)
 *
 *  Evidential FastSLAM - An evidential approach to SLAM
 *  Copyright (c) 2012-2015, Joachim Clemens, Thomas Reineking, Tobias Kluth
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

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <algorithm>

#ifndef WIN32
// Time functions
#	include <time.h>

// Dir access
#	include <sys/types.h>
#	include <sys/stat.h>
#	include <unistd.h>
#	include <errno.h>
#	include <string.h>
#endif

#include "efslam/utils/Log.h"
#include "efslam/utils/Config.h"


namespace efs {

#define KEY_WIDTH 32

#define CHECK_KEY( __key ) if( !isValidKey( __key ) ) throw std::invalid_argument( "invalid characters in key" );

std::unordered_map<std::string, std::string>	Config::sm_stringVals;
std::unordered_map<std::string, double>			Config::sm_doubleVals;
std::unordered_map<std::string, int>			Config::sm_intVals;

enum ParserState {
	SEARCHING_KEY,
	IN_KEY,
	SEARCHING_SEPARATOR,
	IN_VALUE,
	SEARCHING_VALUE,
	IN_COMMENT,
	IN_ESCAPE,
	SEARCHING_END,
	IN_QUOTE,
	ERROR
};


void
Config::loadDefaults( bool clearBeforeLoad ) {
	if( clearBeforeLoad )
		clear();

	// general
	set( "MAP_TYPE", 		"belief" 		);
	set( "FILENAME", 		"intel.clf" );
	set( "GT_FILENAME",		"" 				);
	set( "OUTPUT_BASEDIR",	"../results" 	);
	set( "OUTPUT_DIR",		""				);	// if this value remains empty, it will be overwritten by createOutputDir
	set( "NO_GUI",			0				); 	// only for CarmenSimpleGui

	// Laser scanner
	set( "LASER_START_ANGLE", 			-90.0 		); // [deg] Angle of first laser beam
	set( "LASER_ANGULAR_RES",   		1.0 		); // [deg] Angle between to laser beams

	//------------------------
	// Belief sensor model
	//------------------------
	set( "BELIEF_SIGMA_FORWARD",		0.025	);	// [m]	   standard deviation for forward model
	set( "BELIEF_SIGMA_INVERSE",		0.025	);	// [m]	   standard deviation for inverse model
	set( "BELIEF_SIGMA_FORWARD_FACTOR",	3.0		);	// [0-inf] sigma range (factor * BELIEF_SIGMA) to include in forward model computation (controls the plausibility before obstacles and hence the specificity of the model)
	set( "BELIEF_SIGMA_INVERSE_FACTOR",	3.0		);	// [0-inf] sigma range (factor * BELIEF_SIGMA) to include in inverse model computation (controls the width of the Theta gap between free and occupied
	set( "BELIEF_RANDOM", 				0.2		);	// [0-1]   controls the mass that remains on Theta
	set( "BELIEF_OCC_PRIOR",			0.4 	);	// [0-1]   controls the mass on occupied
	set( "BELIEF_EXCLUDE_NO_HIT",		0		);	// do not use forward model if scan matching failed for this measurement
	set( "BELIEF_CELLS_BEHIND",			3		);	// Number of cells to consider behind the measurement when registering a scan

	//------------------------
	// Bayes sensor model
	//------------------------
	set( "BAYES_SIGMA",  			0.25	);		// [m]		standard deviation for forward model

	//--------------
	// Motion model
	//--------------
	set( "TRANS_TRANS_SIGMA",	0.1 	);	// [m^2]
	set( "TRANS_ROT_SIGMA",		0.1 	);	// [m*rad]
	set( "ROT_ROT_SIGMA",		0.1 	);	// [rad^2]
	set( "ROT_TRANS_SIGMA",		0.1 	);	// [rad*m]


	//--------------
	// SLAM
	//--------------
	set( "LINEAR_DIST_THRESHOLD", 	0.5 	);	// [m]		process the next scan after the robot traveled this distance
	set( "ANGULAR_DIST_THRESHOLD", 	10.0 	);	// [deg]	process the next scan after the robot turned by this angle
	set( "MAP_MIN_X", 				-100.0 	);	// [m]
	set( "MAP_MAX_X", 				100.0 	);	// [m]
	set( "MAP_MIN_Y", 				-100.0 	);	// [m]
	set( "MAP_MAX_Y", 				100.0	);	// [m]
	set( "MAP_DELTA",				0.05	);	// [m] 		size of a cell


	//--------------
	// FastSLAM
	//--------------
	set( "NUM_PARTICLES", 			50 		);
	set( "MIN_SCORE",				0.0		);	// [0-inf]
	set( "RESAMPLE_THRESHOLD",		0.5		);	// [0-1]	percentage of effective particles
	set( "LIKELIHOOD_GAIN",			3.0		);	// (0-inf]	used to compute weights from log weights, higher values reduces frequency of resampling

	//------------------
	// Map Scan matcher
	//------------------
	set( "FREE_CELL_RATIO", 	sqrt( 2.0 )	);	// Distance before the hit cell where an empty cell is expected (will be multiplied with MAP_DELTA)
	set( "USABLE_RANGE", 		15.0 		);	// [m] 	 Greater measurements will be reduced to this value
	set( "MAX_RANGE",			80.0 		);	// [m] 	 Measurements greater than this value will be skipped
	set( "FULLNESS_THRESHOLD", 	0.1 		);	// [0-1] A cell is considered to be occupied if the fullness is above this value and considered to be empty otherwise
	set( "RELEVANT_DIST",		0.0			);	// [m]   Only cells in the given distance from the measurement are considered in forward and inverse model, set to 0 to disable the feature

	// Score calculation
	set( "SCORE_SIGMA", 				0.25 		);
	set( "KERNEL_SIZE", 				1 			);	// Half size of the kernel for score calculation, kernel will be 2*KERNEL_SIZE + 1
	set( "KERNEL_CROSS_2D", 			0 			);	// Use a cross instead of a square as kernel for score calculation in 2D
	set( "LIKELIHOOD_NUM_MEASUREMENTS",	500			);	// Fixed number of measurements to use to compute the likelihood. Note that additional measurements can be skipped when no matching cell is found. If set to 0 or if LIKELIHOOD_LIKE_SCORE=1, SCORE_SKIP_POINTS and SCORE_RANGE_* is used.
	set( "LIKELIHOOD_LIKE_SCORE",		1			);	// Select the points for likelihood calculation like for score calculation, i.e. use SCORE_RANGE_* and SCORE_SKIP_POINTS while LIKELIHOOD_NUM_MEASUREMENTS is ignored

	// Pose optimization
	set( "OPT_ANGULAR_STEP", 	3.0 		);	// [deg]
	set( "OPT_LINEAR_STEP", 	.05 		);	// [m]
	set( "OPT_ITERATIONS",		5 			);
}


void
Config::loadParams( int argc, char *argv[], bool clearBeforeLoad, bool warnUnknown ) {
	if( argc < 3 )
		return;

	if( clearBeforeLoad )
		clear();

	int cur = 1;
	while( (argc - cur) >= 2 ) {
		// params start with "--"
		if( argv[cur][0] != '-' || argv[cur][1] != '-' ) {
			cur++;
			continue;
		}

		bool error = false;

		std::string	key 	= &argv[cur][2],
					value	= argv[cur+1];

		// convert to upper case and '-' to '_'
		for( size_t i = 0; i < key.size(); i++ ) {
			key[i] = std::toupper( key[i] );
			if( key[i] == '-' )
				key[i] = '_';

			if( !isAllowedKeyChar( key[i] ) ) {
				l_wrn( "Unallowed character in command line argument #" << cur << " `" << argv[cur] << "': " << key[i] );
				error = true;
				break;
			}
		}

		if( error ) {
			cur++;
			continue;
		}

		addValue( key, value, false, warnUnknown );

		cur += 2;
	}
}


void
Config::loadStream( std::istream &stream, bool clearBeforeLoad, bool warnUnknown ) {
	std::string line,
				key,
				value,
				error;
	int lineNum 		= 0;
	bool wasQuoted 		= false;
	ParserState state 	= SEARCHING_KEY;

	if( clearBeforeLoad )
		clear();

	// parse lines
	while( std::getline( stream, line ) ) {
		lineNum++;
		for( std::string::iterator c = line.begin(); c != line.end() && state != ERROR && state != IN_COMMENT; c++ ) {
			switch( state ) {
				case SEARCHING_KEY:
					if( *c == '#' ) {
						state = IN_COMMENT;
					} else if( isAllowedKeyChar( *c ) ) {
						key += *c;
						state = IN_KEY;
					}	else if( !isspace( *c ) ) {
						error = std::string( "invalid char for a key: `" ).append( 1, *c ).append( "\'" );
						state = ERROR;
					}
					break;

				case IN_KEY:
					if( *c == ':' ) {
						state = SEARCHING_VALUE;
					} else if( isspace( *c ) ) {
						state = SEARCHING_SEPARATOR;
					} else if( isAllowedKeyChar( *c ) ) {
						key += *c;
					} else {
						error = std::string( "invalid char for a key: `" ).append( 1, *c ).append( "'" );
						state = ERROR;
					}
					break;

				case SEARCHING_SEPARATOR:
					if( *c == ':' ) {
						state = SEARCHING_VALUE;
					} else if( !isspace( *c ) ) {
						error = std::string( "separator `:' expected, not `" ).append( 1, *c ).append( "'" );
						state = ERROR;
					}
					break;

				case SEARCHING_VALUE:
					if( *c == '#' ) {
						error = std::string( "value expected" );
						state = ERROR;
					} else if( *c == '\"' ) {
						state = IN_QUOTE;
					} else if( !isspace( *c ) ) {
						value += *c;
						state = IN_VALUE;
					}
					break;

				case IN_VALUE:
					if( *c == '#' ) {
						state = IN_COMMENT;
					} else if( isspace( *c ) ) {
						state = SEARCHING_END;
					} else {
						value += *c;
					}
					break;

				case IN_QUOTE:
					wasQuoted = true;
					if( *c == '"' ) {
						state = SEARCHING_END;
					} else if( *c == '\\' ) {
						state = IN_ESCAPE;
					} else {
						value += *c;
					}
					break;

				case IN_ESCAPE:
					if( *c == '"' ) {
						value += *c;
						state = IN_QUOTE;
					/* // this don't have to be escaped anymore
					} else if( *c == '#' ) {
						value += *c;
					*/
					} else if( *c == '\\' ) {
						value += '\\';
						state = IN_ESCAPE;
					} else {
						value += '\\';
						value += *c;
						state = IN_QUOTE;
					}
					break;

				case SEARCHING_END:
					if( *c == '#' ) {
						state = IN_COMMENT;
					} else if( !isspace( *c ) ) {
						error = std::string( "unexpected char: `" ).append( 1, *c ).append( "\' (missing quotes?)" );
						state = ERROR;
					}
					break;

				case IN_COMMENT:
				case ERROR:
					break;
				default:
					l_err( "Invalid parser state." );
					break;
			}
		}

		if( state == ERROR ) {
			l_wrn( "Syntax error in line " << lineNum << ": " << error.c_str() );
		} else if( state == IN_QUOTE ) {
			l_wrn( "Syntax error in line " << lineNum << ": missing `\"'" );
		} else if( key.empty() ) {
			// ignore
		} else if( !wasQuoted && value.empty() ) {
			l_wrn( "Syntax error in line " << lineNum << ": key `" << key << "' with empty value" );
		} else {
			addValue( key, value, wasQuoted );
		}

		error.clear();
		key.clear();
		value.clear(); 
		state = SEARCHING_KEY;
		wasQuoted = false;
	}

	//std::cout << toString() << std::endl;
}


void
Config::saveStream( std::ostream &stream ) {
	// write std::strings
	for( auto iter = sm_stringVals.begin(); iter != sm_stringVals.end(); iter++ ) {
		// escape double-quotes
		std::string outStr = iter->second;
		for( size_t pos = 0; pos < outStr.length(); pos++ ) {
			if( outStr[pos] == '"' ) {
				outStr.replace( pos, 1, "\\\"" );
				pos++;
			}
			/* //this don't have to escaped anymore
			else if( outStr[pos] == '#' ) {
				outStr.replace( pos, 1, "\\#" );
				pos++;
			}
			*/
		}

		stream << std::setw( KEY_WIDTH ) << std::left << iter->first + ":" << "\"" << outStr << "\"" << std::endl;
	}

	// write doubles
	for( auto iter = sm_doubleVals.begin(); iter != sm_doubleVals.end(); iter++ )
		stream << std::setw( KEY_WIDTH ) << std::left << iter->first + ":" << std::fixed << std::setprecision(20) << iter->second << std::endl;

	// write ints
	for( auto iter = sm_intVals.begin(); iter != sm_intVals.end(); iter++ )
		stream << std::setw( KEY_WIDTH ) << std::left << iter->first + ":" << iter->second << std::endl;
}


void
Config::loadFile( const char *filename, bool clearBeforeLoad, bool warnUnknown ) {
	std::ifstream filestream( filename );

	if( !filestream.is_open() )
		throw std::ios_base::failure( "Failed to open config file for reading." );

	loadStream( filestream, clearBeforeLoad, warnUnknown );

	filestream.close();
}


void
Config::loadFile( const std::string &filename, bool clearBeforeLoad, bool warnUnknown ) {
	loadFile( filename.c_str(), clearBeforeLoad, warnUnknown );
}


void
Config::loadDefaultFile( bool clearBeforeLoad, bool warnUnknown ) {
	loadFile( std::string( std::getenv( "HOME" ) ) + std::string( "/.efslamrc" ), clearBeforeLoad, warnUnknown );
}

void
Config::saveFile( char const *filename ) {
	// Note: This will overwrite comments, ordering, and formattings in existing files

	std::ofstream filestream( filename );

	if( !filestream.is_open() )
		throw std::ios_base::failure( "Failed to open config file for writing." );
	
	saveStream( filestream );

	filestream.close();
}


void
Config::saveFile( const std::string &filename ) {
	saveFile( filename.c_str() );
}


#ifndef WIN32
void
Config::createOutputDir( const std::string &suffix ) {
	std::string outputDir = get( "OUTPUT_DIR", "" );

	if( outputDir.empty() ) {
		time_t	curtime;
		char	prefix[200];

		time( &curtime );
		strftime( prefix, 199, "%F_%H-%M-%S_", localtime( &curtime ) );

		std::string	basedir	= get( "OUTPUT_BASEDIR", "." );

		outputDir = basedir + "/" + prefix + (!suffix.empty() ? suffix : std::string( "efslam" ) );
	}

	struct stat st;
	if( stat( outputDir.c_str(), &st ) != 0 ) // exists already?
		if( mkdir( outputDir.c_str(), 0755 ) != 0 && errno != EEXIST ) // EEXIST for race condition
			throw std::ios::failure( "Failed to create output directory `" + outputDir + "': " + std::string( strerror( errno ) ) );

	set( "OUTPUT_DIR", outputDir );
}
#endif


void
Config::clear() {
	sm_stringVals.clear();
	sm_intVals.clear();
	sm_doubleVals.clear();
}


std::string
Config::getString( const std::string &key ) {
	return sm_stringVals.at( key );
}


std::string
Config::getString( const std::string &key, const char *defaultVal ) {
	std::string val;

	try {
		val = getString( key );
	} catch( std::out_of_range &oor ) {
		val = std::string( defaultVal );
		l_wrn( "Unable to retrieve std::string value for `" << key.c_str() << "' from configuration, using default `" << defaultVal << "'." );
	}

	return val;
}


std::string
Config::getString( const std::string &key, const std::string defaultVal ) {
	return getString( key, defaultVal.c_str() );
}


double
Config::getDouble( const std::string &key ) {
	return sm_doubleVals.at( key );
}


double
Config::getDouble( const std::string &key, const double defaultVal ) {
	double val;

	try {
		val = getDouble( key );
	} catch( std::out_of_range &oor ) {
		val = defaultVal;
		l_wrn( "Unable to retrieve double value for `" << key.c_str() << "' from configuration, using default `" << defaultVal << "'." );
	}

	return val;
}


int
Config::getInt( const std::string &key ) {
	return sm_intVals.at( key );
}


int
Config::getInt( const std::string &key, const int defaultVal ) {
	int val;

	try {
		val = getInt( key );
	} catch( std::out_of_range &oor ) {
		val = defaultVal;
		l_wrn( "Unable to retrieve int value for `" << key.c_str() << "' from configuration, using default `" << defaultVal << "'." );
	}

	return val;
}


std::string
Config::get( const std::string &key, const std::string defaultVal ) {
	return getString( key, defaultVal );
}


double
Config::get( const std::string &key, const double defaultVal ) {
	return getDouble( key, defaultVal );
}


int
Config::get( const std::string &key, const int defaultVal ) {
	return getInt( key, defaultVal );
}


void
Config::set( const std::string &key, const std::string val ) {
	CHECK_KEY( key );
	sm_stringVals[key] = val;
}


void
Config::set( const std::string &key, const double val ) {
	CHECK_KEY( key );
	sm_doubleVals[key] = val;
}


void
Config::set( const std::string &key, int val ) {
	CHECK_KEY( key );
	sm_intVals[key] = val;
}


bool
Config::hasStringValue( const std::string &key ) {
  return sm_stringVals.find( key ) != sm_stringVals.end();
}


bool
Config::hasDoubleValue( const std::string &key ) {
  return sm_doubleVals.find( key ) != sm_doubleVals.end();
}


bool
Config::hasIntValue( const std::string &key ) {
	return sm_intVals.find( key ) != sm_intVals.end();
}


std::string
Config::toString() {
	std::ostringstream strs;

	strs << "Strings:" << std::endl;
	for( auto iter = sm_stringVals.begin(); iter != sm_stringVals.end(); iter++ )
		strs << std::setw( KEY_WIDTH ) << std::left << iter->first + " ->" << "`" << iter->second << "'" << std::endl;

	strs << std::endl << "Doubles:" << std::endl;
	for( auto iter = sm_doubleVals.begin(); iter != sm_doubleVals.end(); iter++ )
		strs << std::setw( KEY_WIDTH ) << std::left << iter->first + " ->" << std::fixed << std::setprecision(20) << iter->second << std::endl;

	strs << std::endl << "Integers:" << std::endl;
	for( auto iter = sm_intVals.begin(); iter != sm_intVals.end(); iter++ )
		strs << std::setw( KEY_WIDTH ) << std::left << iter->first + " ->" << iter->second << std::endl;

	return strs.str();
}


bool
Config::isValidKey( const std::string &key ) {
	for( size_t i = 0; i < key.size(); i++ )
		if( !isAllowedKeyChar( key[i] ) )
			return false;
	
	return true;
}


bool
Config::isAllowedKeyChar( char c ) {
	if( isalnum( c ) || c == '_' || c == '-' )
		return true;
	else
		return false;
}


Config::ValueType
Config::getValueType( const std::string &str ) {
	ValueType type = VALUE_TYPE_INT;

	for( size_t i = 0; i < str.length(); i++ ) {
		//std::cout << i << " " << str[i] << std::endl;
		if( type == VALUE_TYPE_INT && str[i] == '.' ) // found dot in number
			type = VALUE_TYPE_DOUBLE;
		else if( (type == VALUE_TYPE_DOUBLE && str[i] == '.')					// found a second dot
					|| (!isdigit( str[i] ) && str[i] != '.' && str[i] != '-' ) 	// not a digit and not a point
					|| (str[i] == '-' && i > 0 )								// minus not as the first char
						) {
			type = VALUE_TYPE_STRING;
			break;
		}
	}

	//std::cout << "Result: " << type << std::endl;

	return type;
}


void
Config::addValue( const std::string &key, const std::string &value, bool wasQuoted, bool warnUnknown ) {
	ValueType type = getValueType( value );
	if( wasQuoted || type == VALUE_TYPE_STRING ) {
		if( warnUnknown && sm_stringVals.find( key ) == sm_stringVals.end() )
			l_wrn( "Key `" << key << "' was previously unknown for value type string!" );
		sm_stringVals[key]	= value;
	} else if (type == VALUE_TYPE_INT ) {
		if( warnUnknown && sm_intVals.find( key ) == sm_intVals.end() )
			l_wrn( "Key `" << key << "' was previously unknown for value type int!" );
		sm_intVals[key]		= std::stoi( value );
	} else {
		if( warnUnknown && sm_doubleVals.find( key ) == sm_doubleVals.end() )
			l_wrn( "Key `" << key << "' was previously unknown for value type double!" );
		sm_doubleVals[key]	= std::stof( value );
	}
}


std::string
Config::toLower( const std::string &in ) {
	std::string out( in );

	std::transform( out.begin(), out.end(), out.begin(), ::tolower );

	return out;
}


std::string
Config::toUpper( const std::string &in ) {
	std::string out( in );

	std::transform( out.begin(), out.end(), out.begin(), ::toupper );

	return out;
}

} /* namespace efs */

