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

#ifndef EFS_CONVENIENCE_H_
#define EFS_CONVENIENCE_H_

#include <algorithm>
#include <string>


namespace efs {

// Should be more efficient than the defines
template<typename T>
inline constexpr T
SQR( const T &x ) {
	return x*x;
}


template<typename T>
inline constexpr T
POW3( const T &x ) {
	return x*x*x;
}


template<typename T>
inline constexpr T
POW4( const T &x ) {
	return x*x*x*x;
}


template<typename T>
inline constexpr T
POW5( const T &x ) {
	return x*x*x*x*x;
}

#ifndef CWISE_SQR
#	define CWISE_SQR( __x )                ( (__x).cwiseProduct( (__x) ) )
#endif

/*
#define ANGLE_NORMALIZE( __alpha )      ( atan2( sin( (__alpha) ), cos( (__alpha) ) ) )
#define ANGLE_DIFF( __alpha, __beta )   ( ANGLE_NORMALIZE( (__alpha) - (__beta) ) )
*/


template<typename T>
inline T&
ANGLE_NORMALIZE_INPLACE( T &a ) {
	if( a >= -M_PI && a < M_PI )
		return a;

	a = fmod( a, 2*M_PI );

	if( a >= M_PI )
		a -= 2*M_PI;

	if( a < -M_PI )
		a += 2*M_PI;

	return a;
}


template<typename T>
inline T
ANGLE_NORMALIZE( T a ) {
	return ANGLE_NORMALIZE_INPLACE( a );
}


/*
template<typename T>
inline constexpr T
ANGLE_NORMALIZE( const T &a ) {
	//return atan2( sin( a ), cos( a ) );

	// This only works for positive angles!
	//return fmod( a + M_PI, 2*M_PI ) - M_PI;
}
*/


template<typename T>
inline constexpr T
ANGLE_DIFF( const T &a, const T &b ) {
	return ANGLE_NORMALIZE( a - b );
}


template<typename T>
inline constexpr T
MAX( const T &x, const T &y ) {
	return (x) > (y) ? (x) : (y);
}


template<typename T>
inline constexpr T
MIN( const T &x, const T &y ) {
	return (x) < (y) ? (x) : (y);
}

#ifndef SIGN
#	define SIGN( x )					( (x) >= 0  ?  1  :  -1 )
#endif

#ifndef DEG2RAD
#	define DEG2RAD( a )					( (a) * M_PI / 180.0 )
#endif
#ifndef RAD2DEG
#	define RAD2DEG( a )					( (a) * 180.0 / M_PI )
#endif

/*
template< size_t N >
Eigen::Matrix<double, N, 1>
cwiseAngleDiff( Eigen::Matrix<double, N, 1> alpha, Eigen::Matrix<double, N, 1> beta )
{
    Eigen::Matrix<double, N, 1> res,
                                diff = alpha - beta;

    for( size_t i = 0; i < N; i++ )
        res[i] = ANGLE_DIFF_D( diff[i] );

    return res;
}
*/

// auto as return type for functions without trailing return type syntax is supported
// since c++14, but not in c++11, so we have to use decltype here and the getters and
// setters have to be defined after member variable declaration.
#ifndef SETTER
#	define SETTER( var )           inline       decltype( m_ ## var )&     var()       { return m_ ## var; }
#endif
#ifndef GETTER
#	define GETTER( var )          	inline const decltype( m_ ## var )&     var() const { return m_ ## var; }
#endif

// for static members, only one of them can be used
#ifndef S_SETTER
#	define S_SETTER( var )  static  inline       decltype( sm_ ## var )&     var() { return sm_ ## var; }
#endif
#ifndef S_GETTER
#	define S_GETTER( var )  static  inline const decltype( sm_ ## var )&     var() { return sm_ ## var; }
#endif


/*
 THIS DOES NOT WORK ON WINDOWS
namespace std {

// generic hash function for enums to be used with std containers

template<class E>
class hash
{
using ENotAnEnum = typename std::enable_if< std::is_enum<E>::value, E >::type;
public:
    size_t operator()( const E &e ) const
    {
      return std::hash<typename std::underlying_type<E>::type>()( e );
    }
};

}
*/


} /* namespace efs */


namespace std {

// Forward declarations
template<typename T, typename Allocator>	class vector;

}



namespace efs {

class specialization_need : public std::exception {
	virtual const char* what() const throw() {
		return "A specialized implementation for this typename is needed.";
	}
};


template< class T >
inline std::string
to_string( const T &t ) {
	std::stringstream ss;
	ss << t;
	return ss.str();
}


inline std::string
to_lower( std::string s ) {
	std::transform( s.begin(), s.end(), s.begin(), tolower );
	return s;
}


std::vector<std::string>
split( const std::string &s, char delim );


} /* namespace efs */

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif


#endif /* EFS_CONVENIENCE_H_ */
