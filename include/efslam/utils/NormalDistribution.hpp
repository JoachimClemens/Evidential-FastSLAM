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
#include <functional>
#include <random>

#include "Convenience.h"

namespace efs {


template< size_t N >
Eigen::Matrix<double, N, N>
NormalDistribution::cov2normTransform( const Eigen::Matrix<double, N, N> &cov ) {
    // according to http://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
    // and https://forum.kde.org/viewtopic.php?f=74&t=95260
    // Second part of the sampling is in drawWithNormTrans()

    Eigen::Matrix<double, N, N>                 normTransform;
    Eigen::LLT< Eigen::Matrix<double, N, N> >   cholSolver( cov );

    // We can only use the cholesky decomposition if
    // the covariance matrix is symmetric, pos-definite.
    // But a covariance matrix might be pos-semi-definite.
    // In that case, we'll go to an EigenSolver
    if( cholSolver.info() == Eigen::Success ) {
      // Use cholesky solver
      normTransform = cholSolver.matrixL();
    } else {
      // Use eigen solver
      Eigen::SelfAdjointEigenSolver< Eigen::Matrix<double, N, N> > eigenSolver( cov );
      normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    return normTransform;
}


double
NormalDistribution::drawStddev( double mean, double stddev ) {
	static std::random_device 					rd;
	static std::mt19937 						generator( rd() );
    static std::normal_distribution<double>     distribution( 0.0, 1.0 );

    return mean + distribution( generator ) * stddev;
}


template< size_t N >
Eigen::Matrix<double, N, Eigen::Dynamic>
NormalDistribution::drawWithNormTrans( const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &normTransform, size_t numSamples ) {
    // see cov2normTransform() for the first part of the sampling process and further references

    static  auto randN = []( size_t __indexIsIgnored ) { return draw( 1.0 ); };

    return ( normTransform * Eigen::Matrix<double, N, Eigen::Dynamic>::NullaryExpr( N, numSamples, randN ) ).colwise() + mean;
}


double
NormalDistribution::logPdf( double x, double mean, double var ) {
    return log( pdfScale( var ) ) + logPl( x, mean, var );
}


double
NormalDistribution::pdf( double x, double mean, double var ) {
    return pdfScale( var ) * pl( x, mean, var );
}


double
NormalDistribution::pdfStddev( double x, double mean, double stddev ) {
	return pdfScaleStddev( stddev ) * plStddev( x, mean, stddev );
}


double
NormalDistribution::pdfScale( double var ) {
	return pdfScaleStddev( sqrt( var ) );
}


double
NormalDistribution::pdfScaleStddev( double stddev ) {
	return 1.0 / ( stddev * sm_sqrtTwoPi );
}


template< size_t N >
double
NormalDistribution::logPdf( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov ) {
    return log( pdfScale<N>( cov ) ) + logPl<N>( x, mean, cov );
}


template< size_t N >
double
NormalDistribution::pdf( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov ) {
    return pdfScale<N>( cov ) * pl<N>( x, mean, cov );
}


template< size_t N >
double
NormalDistribution::pdfScale( const Eigen::Matrix<double, N, N> &cov ) {
    return 1.0 / sqrt( pow( 2*M_PI, N ) * cov.determinant() );
}




double
NormalDistribution::cdf( double x ) {
	// According to http://www.johndcook.com/blog/cpp_phi/

    // constants
    static const double a1 =  0.254829592;
    static const double a2 = -0.284496736;
    static const double a3 =  1.421413741;
    static const double a4 = -1.453152027;
    static const double a5 =  1.061405429;
    static const double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if( x < 0 )
        sign = -1;
    x = fabs( x ) / sqrt( 2.0 );

    // Abramowitz and Stegun, Handbook of Mathematical Functions, formula 7.1.26
    double t = 1.0 / ( 1.0 + p*x );
    double y = 1.0 - ( ( ( ( (a5*t + a4)*t ) + a3 )*t + a2 )*t + a1 ) * t * exp( -x*x );

    return 0.5*(1.0 + sign*y);
}


template< size_t N >
double
NormalDistribution::logPl( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov ) {
    Eigen::Matrix<double, N, 1> diff = x - mean;
    return -0.5 * diff.transpose() * cov.inverse() * diff;
}


template< size_t N >
double
NormalDistribution::pl( const Eigen::Matrix<double, N, 1> &x, const Eigen::Matrix<double, N, 1> &mean, const Eigen::Matrix<double, N, N> &cov ) {
    return exp( logPl<N>( x, mean, cov ) );
}


template< typename SamplesContainer >
double
NormalDistribution::mean( const SamplesContainer &samples ) {
    double  res = 0;

    for( size_t i = 0; i < samples.size(); i++ )
        res += samples[i];

    return res / samples.size();
}


template< typename SamplesContainer, typename WeightsVector >
double
NormalDistribution::mean( const SamplesContainer &samples, const WeightsVector &weights ) {
    double  res = 0;

    for( size_t i = 0; i < samples.size(); i++ )
        res += weights[i] * samples[i];

    return res;
}


template< size_t N, typename SamplesContainer >
Eigen::Matrix<double, N, 1>
NormalDistribution::mean( const SamplesContainer &samples ) {
    Eigen::Matrix<double, N, 1> res;
    mean<N, SamplesContainer>( &res, samples );
    return res;
}


template< size_t N, typename SamplesContainer >
void
NormalDistribution::mean( Eigen::Matrix<double, N, 1> *res, const SamplesContainer &samples ) {
    res->setZero();

    for( size_t i = 0; i < samples.size(); i++ )
        (*res) += samples[i];

    (*res) /= samples.size();
}



template< size_t N, typename SamplesContainer, typename WeightsVector >
Eigen::Matrix<double, N, 1>
NormalDistribution::mean( const SamplesContainer &samples, const WeightsVector &weights ) {
    Eigen::Matrix<double, N, 1> res;
    mean<N, SamplesContainer, WeightsVector>( &res, samples, weights );
    return res;
}


template< size_t N, typename SamplesContainer, typename WeightsVector >
Eigen::Matrix<double, N, 1>
NormalDistribution::meanPtr( const SamplesContainer &samples, const WeightsVector &weights ) {
    Eigen::Matrix<double, N, 1> res;
    meanPtr<N, SamplesContainer, WeightsVector>( &res, samples, weights );
    return res;
}



template< size_t N, typename SamplesContainer, typename WeightsVector >
void
NormalDistribution::mean( Eigen::Matrix<double, N, 1> *res, const SamplesContainer &samples, const WeightsVector &weights ) {
    res->setZero();

    for( size_t i = 0; i < samples.size(); i++ )
        (*res) += weights[i] * samples[i];
}


template< size_t N, typename SamplesContainer, typename WeightsVector >
void
NormalDistribution::meanPtr( Eigen::Matrix<double, N, 1> *res, const SamplesContainer &samples, const WeightsVector &weights ) {
    res->setZero();

    for( size_t i = 0; i < samples.size(); i++ )
        (*res) += weights[i] * *samples[i];
}


template< typename SamplesContainer >
double
NormalDistribution::var( double mean, const SamplesContainer &samples ) {
    double res = 0;

    for( size_t i = 0; i < samples.size(); i++ )
    {
        double diff = samples[i] - mean;
        res += SQR( diff );
    }

    return res / samples.size();
}


template< typename SamplesContainer, typename WeightsVector >
double
NormalDistribution::var( double mean, const SamplesContainer &samples, const WeightsVector &weights ) {
    double res = 0;

    for( size_t i = 0; i < samples.size(); i++ )
    {
        double diff = samples[i] - mean;
        res += weights[i] * SQR( diff );
    }

    return res;
}


template< size_t N, typename SamplesContainer >
Eigen::Matrix<double, N, N>
NormalDistribution::cov( const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples ) {
    Eigen::Matrix<double, N, N> res;
    cov<N, SamplesContainer>( &res, mean, samples );
    return res;
}


template< size_t N, typename SamplesContainer >
void
NormalDistribution::cov( Eigen::Matrix<double, N, N> *res, const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples ) {
    res->setZero();
    for( size_t i = 0; i < samples.size(); i++ ) {
        Eigen::Matrix<double, N, 1> diff    =   samples[i] - mean;
        (*res)                              +=  (diff * diff.transpose());
    }

    (*res) /= samples.size();
}


template< size_t N, typename SamplesContainer, typename WeightsVector >
Eigen::Matrix<double, N, N>
NormalDistribution::cov( const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples, WeightsVector &weights ) {
    Eigen::Matrix<double, N, N> res;
    cov<N, SamplesContainer, WeightsVector>( &res, mean, samples, weights );
    return res;
}


template< size_t N, typename SamplesContainer, typename WeightsVector >
void
NormalDistribution::cov( Eigen::Matrix<double, N, N> *res, const Eigen::Matrix<double, N, 1> &mean, const SamplesContainer &samples, WeightsVector &weights ) {
    res->setZero();
    for( size_t i = 0; i < samples.size(); i++ ) {
        Eigen::Matrix<double, N, 1> diff    =   samples[i] - mean;
        (*res)                              +=  weights[i] * (diff * diff.transpose());
    }
}

} /* namespace efs */
