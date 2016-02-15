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

#ifndef EFS_CONFIG_H_
#define EFS_CONFIG_H_

#include <unordered_map>
#include <string>


namespace efs {

class Config {
public:
	static	void		loadDefaults( bool clearBeforeLoad = false );

	static	void		loadParams( int argc, char *argv[], bool clearBeforeLoad = false, bool warnUnknown = true );

	static	void		loadStream( std::istream &f, bool clearBeforeLoad = false, bool warnUnknown = true );
	static	void		saveStream( std::ostream &f );

	static	void		loadFile( char const *filename,  bool clearBeforeLoad = false, bool warnUnknown = true );
	static	void		loadFile( const std::string &filename,  bool clearBeforeLoad = false, bool warnUnknown = true );
	static	void		saveFile( char const *filename );
	static	void		saveFile( const std::string &filename );

	static	void		loadDefaultFile( bool clearBeforeLoad = false, bool warnUnknown = true );

#ifndef WIN32
	static	void		createOutputDir( const std::string &suffix = "" );
#endif

	static	std::string	getString( const std::string &key );
	static	std::string	getString( const std::string &key, const char *defaultVal );
	static	std::string	getString( const std::string &key, const std::string defaultVal );
	static	double		getDouble( const std::string &key );
	static	double		getDouble( const std::string &key, const double defaultVal );
	static	int			getInt( const std::string &key );
	static	int			getInt( const std::string &key, const int defaultVal );

	static	std::string	get( const std::string &key, const std::string defaultVal );
	static	double		get( const std::string &key, const double defaultVal );
	static	int			get( const std::string &key, const int defaultVal );

	static	void		set( const std::string &key, const std::string val );
	static	void		set( const std::string &key, const double val );
	static	void		set( const std::string &key, int val );

	static	bool		hasStringValue( const std::string &key );
	static	bool		hasDoubleValue( const std::string &key );
	static	bool		hasIntValue( const std::string &key );

	static	bool		isValidKey( const std::string &key );

	static	std::string	toString();

private:
						Config(); // we don't want any instance of this class

	enum ValueType {
		VALUE_TYPE_STRING,
		VALUE_TYPE_INT,
		VALUE_TYPE_DOUBLE
	};

	static	void		clear();

	static	bool		isAllowedKeyChar( char c );
	static	ValueType	getValueType( const std::string &str );
	static	void		addValue( const std::string &key, const std::string &value, bool wasQuoted = false, bool warnUnknown = true );

	static	std::string	toLower( const std::string &in );
	static	std::string	toUpper( const std::string &in );

	static	std::unordered_map<std::string, std::string>	sm_stringVals;
	static	std::unordered_map<std::string, double>			sm_doubleVals;
	static	std::unordered_map<std::string, int>			sm_intVals;
};

} /* namespace efs */

#endif /* EFS_CONFIG_H_ */

