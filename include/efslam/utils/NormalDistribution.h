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

#ifndef EFS_NORMALDISTRIBUTION_H_
#define EFS_NORMALDISTRIBUTION_H_

#include <Eigen/Dense>


namespace efs {

class NormalDistribution
{
public:
    /*----------------------------------*
     * Draw a sample
     *----------------------------------*/

    /** draw with mean and standard deviation */
    static  inline  double  drawStddev( double mean, double stddev );
    /** draw with zero mean and standard deviation */
    static  inline  double  drawStddev( double stddev )                         { return drawStddev( 0.0, stddev );   };

    static  inline  Eigen::Vector2d drawStddev( Eigen::Vector2d stddev )        { return Eigen::Vector2d( drawStddev( 0.0, (double)stddev[0] ), drawStddev( 0.0, (double)stddev[1] ) ); }

    static  inline  Eigen::Vector3d drawStddev( Eigen::Vector3d stddev )        { return Eigen::Vector3d( drawStddev( 0.0, (double)stddev[0] ), drawStddev( 0.0, (double)stddev[1] ), drawStddev( 0.0, (double)stddev[2] ) ); }

    /** draw with mean and variance */
    static  inline  double  draw( double mean, double var )                     { return drawStddev( mean, sqrt( var ) ); }
    /** draw with zero mean and variance */
    static  inline  double  draw( double var )                                  { return draw( 0.0, var );   }

    /** Compute the normal transformation from the covariance needed for mutlivariate sampling
     * When needed to sample multiple times with the same covariance, it is a lot of faster
     * to compute the transformation separately and use it for sampling, because computing it
     * is the most expensive part of the sampling process and not the sample generation itself.
     */
    template< size_t N >
    static	inline	Eigen::Matrix<double, N, N>                 cov2normTransform( const Eigen::Matrix<double, N, N> &cov );

    /** draw with mean and precomputed normal transformation (multivariate) */
    template< size_t N >
    static	inline	Eigen::Matrix<double, N, Eigen::Dynamic>    drawWithNormTrans( const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &normTransform, size_t numSamples = 1 );

    /** draw with zero mean and precomputed normal transformation (multivariate) */
    template< size_t N >
    static  inline  Eigen::Matrix<double, N, Eigen::Dynamic>    drawWithNormTrans( const Eigen::Matrix<double, N, N> &normTransform, size_t numSamples = 1 ) {
        return drawWithNormTrans<N>( Eigen::Matrix<double, N, 1>::Zero(), normTransform, numSamples );
    }

    /** draw with mean and covariance (multivariate) */
    template< size_t N >
    static  inline  Eigen::Matrix<double, N, Eigen::Dynamic>    draw( const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov, size_t numSamples = 1 ) {
        return drawWithNormTrans<N>( mean, cov2normTransform<N>( cov ), numSamples );
    }

    /** draw with zero mean and covariance (multivariate) */
    template< size_t N >
    static  inline  Eigen::Matrix<double, N, Eigen::Dynamic>    draw( const Eigen::Matrix<double, N, N> &cov, size_t numSamples = 1 ) {
        return draw<N>( Eigen::Matrix<double, N, 1>::Zero(), cov, numSamples );
    }


    /*----------------------------------*
     * Calculate PDF
     *----------------------------------*/

    /** logarithmic probability density function with mean and variance */
    static 	inline	double  logPdf( double x, double mean, double var );
    /** logarithmic probability density function with zero mean and variance */
    static  inline  double  logPdf( double x, double var )                      { return logPdf( x, 0.0, var ); };

    /** probability density function with mean and variance */
    static 	inline	double  pdf( double x, double mean, double var );
    /** probability density function with zero mean and variance */
    static  inline  double  pdf( double x, double var )                         { return pdf( x, 0.0, var ); };

    /** probability density function with mean and standard deviation */
    static  inline  double  pdfStddev( double x, double mean, double stddev );
    /** probability density function with zero mean and standard deviation */
    static  inline  double  pdfStddev( double x, double stddev )                { return pdfStddev( x, 0.0, stddev ); }

    /** scaling factor to convert pl to pdf for variance */
    static	inline	double	pdfScale( double var );
    /** scaling factor to convert pl to pdf for standard deviation */
    static	inline	double	pdfScaleStddev( double stddev );

    /** logarithmic probability density function (multivariate) */
    template< size_t N >
    static 	inline	double  logPdf( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov );
    /** logarithmic probability density function with zero mean (multivariate) */
    template< size_t N >
    static  inline  double  logPdf( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, N> &cov ) {
        return logPdf<N>( x, Eigen::Matrix<double, N, 1>::Zero(), cov );
    };

    /** probability density function (multivariate) */
    template< size_t N >
    static 	inline	double  pdf( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov );
    /** probability density function with zero mean (multivariate) */
    template< size_t N >
    static  inline  double  pdf( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, N> &cov ) {
        return pdf<N>( x, Eigen::Matrix<double, N, 1>::Zero(), cov );
    };

    /** scaling factor to convert pl to pdf (multivariate) */
    template< size_t N >
    static 	inline	double  pdfScale( const Eigen::Matrix<double, N, N> &cov );


    /*----------------------------------*
     * Calculate CDF
     *----------------------------------*/

    /** cumulative distribution function for zero mean and 1 sigma */
    static	inline	double	cdf( double x );

    /** cumulative distribution function with mean and standard deviation */
    static  inline  double  cdfStddev( double x, double mean, double stddev )   { return cdf( (x - mean) / stddev ); }
    /** cumulative distribution function with zero mean and standard deviation */
    static  inline  double  cdfStddev( double x, double stddev )                { return cdf( x / stddev ); }

    /** cumulative distribution function with mean and variance */
    static 	inline	double  cdf( double x, double mean, double var )			{ return cdfStddev( x, mean, sqrt( var ) ); }
    /** cumulative distribution function with zero mean and variance */
    static  inline  double  cdf( double x, double var )                         { return cdfStddev( x, sqrt( var ) ); }


    /*----------------------------------*
     * Calculate plausibility
     *----------------------------------*/

    /** log plausibility with zero mean and variance for x^2 */
    static 	inline  double  logPlFromSqr( double xSqr, double var )				{ return -xSqr / (2.0 * var); }

    /** log plausibility with zero mean and variance */
    static 	inline  double  logPl( double x, double var )						{ return logPlFromSqr( x*x, var ); }
    /** log plausibility with mean and variance */
    static	inline	double  logPl( double x, double mean, double var )			{ return logPl( x - mean, var ); }

    /** plausibility with zero mean and variance for x^2 */
    static 	inline  double  plFromSqr( double xSqr, double var )				{ return exp( logPlFromSqr( xSqr, var ) ); }

    /** plausibility with zero mean and variance */
    static 	inline  double  pl( double x, double var )							{ return exp( logPl( x, var ) ); }
    /** plausibility with mean and variance */
    static	inline	double  pl( double x, double mean, double var )				{ return pl( x - mean, var ); }

    /** plausibility with mean and standard deviation */
    static 	inline  double  plStddev( double x, double mean, double stddev )	{ return pl( x, mean, stddev*stddev ); };
    /** plausibility with zero mean and standard deviation */
    static 	inline  double  plStddev( double x, double stddev )					{ return plStddev( x, 0.0, stddev );  };

    /** log plausibility (multivariate) */
    template< size_t N >
    static	inline	double  logPl( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov );
    /** log plausibility with zero mean (multivariate) */
    template< size_t N >
    static  inline  double  logPl( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, N> &cov ) {
        return logPl<N>( x, Eigen::Matrix<double, N, 1>::Zero(), cov );
    };

    /** plausibility (multivariate) */
    template< size_t N >
    static	inline	double  pl( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov );
    /** plausibility with zero mean (multivariate) */
    template< size_t N >
    static  inline  double  pl( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, N> &cov ) {
        return pl<N>( x, Eigen::Matrix<double, N, 1>::Zero(), cov );
    };


    /*----------------------------------*
     * Estimate parameters
     *----------------------------------*/

    /** expected value */
    template< typename SamplesContainer >
    static  inline  double  mean( const SamplesContainer &samples );
    /** expected value (weighted samples) */
    template< typename SamplesContainer, typename WeightsVector >
    static  inline  double  mean( const SamplesContainer &samples, const WeightsVector &weights );

    /** variance */
    template< typename SamplesContainer >
    static  inline  double  var( double mean, const SamplesContainer &samples );
    /** variance (weighted samples) */
    template< typename SamplesContainer, typename WeightsVector >
    static  inline  double  var( double mean, const SamplesContainer &samples, const WeightsVector &weights );

    /** expected value (multivariate) */
    template< size_t N, typename SamplesContainer >
    static  inline  Eigen::Matrix<double, N, 1> mean( const SamplesContainer &samples );
    /** expected value (multivariate) */
    template< size_t N, typename SamplesContainer >
    static  inline  void                        mean( Eigen::Matrix<double, N, 1> *res, const SamplesContainer &samples );
    /** expected value (multivariate, weighted samples) */
    template< size_t N, typename SamplesContainer, typename WeightsVector >
    static  inline  Eigen::Matrix<double, N, 1> mean( const SamplesContainer &samples, const WeightsVector &weights );
    /** expected value where SamplesContainer stores pointers (multivariate, weighted samples) */
    template< size_t N, typename SamplesContainer, typename WeightsVector >
    static  inline  Eigen::Matrix<double, N, 1> meanPtr( const SamplesContainer &samples, const WeightsVector &weights );
    /** expected value (multivariate, weighted samples) */
    template< size_t N, typename SamplesContainer, typename WeightsVector >
    static  inline  void                        mean( Eigen::Matrix<double, N, 1> *res, const SamplesContainer &samples, const WeightsVector &weights );
    /** expected value where SamplesContainer stores pointers (multivariate, weighted samples) */
    template< size_t N, typename SamplesContainer, typename WeightsVector >
    static  inline  void                        meanPtr( Eigen::Matrix<double, N, 1> *res, const SamplesContainer &samples, const WeightsVector &weights );

    /** covariance (multivariate) */
    template< size_t N, typename SamplesContainer >
    static  inline  Eigen::Matrix<double, N, N> cov( const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples );
    /** covariance (multivariate) */
    template< size_t N, typename SamplesContainer >
    static  inline  void                        cov( Eigen::Matrix<double, N, N> *res, const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples );
    /** covariance (multivariate, weighted samples) */
    template< size_t N, typename SamplesContainer, typename WeightsVector >
    static  inline  Eigen::Matrix<double, N, N> cov( const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples, WeightsVector &weights );
    /** covariance (multivariate, weighted samples) */
    template< size_t N, typename SamplesContainer, typename WeightsVector >
    static  inline  void                        cov( Eigen::Matrix<double, N, N> *res, const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples, WeightsVector &weights );

private:
    static	constexpr	double sm_sqrtTwoPi	= sqrt( 2 * M_PI );
};


} /* namespace efs */

#include "NormalDistribution.hpp"

#endif /* EFS_NORMALDISTRIBUTION_H_ */
