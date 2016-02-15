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

#include <cmath>


namespace efs {

BeliefFunction::BeliefFunction() :
	Theta(1),
	o(0),
	e(0)
{
	// Nothing else to do here
};


BeliefFunction::BeliefFunction( float _t, float _o, float _e ) :
	Theta(_t),
	o(_o),
	e(_e)
{
	// Nothing else to do here
};


float
BeliefFunction::emptyset() const {
	float sum = 0;
	for( size_t i = 0; i < 3; i++ )
		sum += m[i];
	return 1 - sum;
}


float
BeliefFunction::normalize() {
	float es = emptyset();
	if(  es != 0 )
		for( size_t i = 0; i < 3; i++ )
			m[i] /= (1 - es);
	return es;
}


BeliefFunction
BeliefFunction::normalized() const {
	BeliefFunction res = *this;
	res.normalize();
	return res;
}


void
BeliefFunction::conjunctive( const BeliefFunction &other ) {
	o     = o*other.o + o*other.Theta + Theta*other.o;
	e     = e*other.e + e*other.Theta + Theta*other.e;
	Theta = Theta*other.Theta;
}


void
BeliefFunction::dempster( const BeliefFunction &other ) {
	conjunctive( other );
	normalize();
}


float
BeliefFunction::pignistic() const {
	// returns the value for occupied. The value for empty is implicitly given by 1-o
	return (o + Theta*0.5) / (1 - emptyset());
}


float
BeliefFunction::internalConflict() const {
	// Internal conflict, according to Reineking & Clemens 2014, equations (15) and (16)
	BeliefFunction	mn			= *this;
	double 			emptyset	= mn.normalize(),	// normalize mn and emptyset is returned
					ic			= 0;

	if( mn.o != 0 )
		ic += mn.o * log2(mn.o + mn.Theta);
	if( mn.e != 0 )
		ic += mn.e * log2(mn.e + mn.Theta);

	return -(1 - emptyset) * ic;
}


void
BeliefFunction::set( float Theta_, float o_, float e_ ) {
	Theta 	= Theta_;
	o		= o_;
	e		= e_;
}


bool
BeliefFunction::operator==( const BeliefFunction &other ) const noexcept {
	return Theta == other.Theta && o == other.o && e == other.e;
}


bool
BeliefFunction::operator!=( const BeliefFunction &other ) const noexcept {
	return !(*this == other);
}


} /* namespace efs */
