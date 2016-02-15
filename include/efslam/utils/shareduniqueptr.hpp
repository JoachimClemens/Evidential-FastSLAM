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

namespace efs {

template<typename T>
shared_unique_ptr<T>::shared_unique_ptr() {
	// Nothing to do here
}


template<typename T>
shared_unique_ptr<T>::shared_unique_ptr( T *p ) :
	ptr( std::make_shared<Content>( p ) )
{
	// Nothing else to do here
}


template<typename T>
shared_unique_ptr<T>::shared_unique_ptr( const shared_unique_ptr<T> &other ) :
	ptr( other.ptr )
{
	// Nothing else to do here
}


template<typename T>
shared_unique_ptr<T>::shared_unique_ptr( shared_unique_ptr<T> &&other ) :
	ptr( std::move( other.ptr ) )
{
	// Nothing else to do here
}


template<typename T>
shared_unique_ptr<T>::~shared_unique_ptr() {
	// Nothing to do here
}


template<typename T>
shared_unique_ptr<T>&
shared_unique_ptr<T>::operator=( const shared_unique_ptr<T> &other ) {
	ptr = other.ptr;
	return *this;
}


template<typename T>
shared_unique_ptr<T>&
shared_unique_ptr<T>::operator=( shared_unique_ptr<T> &&other ) {
	ptr = std::move( other.ptr );
	return *this;
}

/*
template<typename T>
void
shared_unique_ptr<T>::make_unique() {
	if( ptr ) {
		std::shared_ptr<std::mutex> mutex = ptr->mutex;

		mutex->lock();
		if( !unique() )
			ptr = std::make_shared<Content>( *ptr );	// create a copy of the content
		mutex->unlock();
	}
}

template<typename T>
void
shared_unique_ptr<T>::make_unique_clone() {
	if( ptr ) {
		std::shared_ptr<std::mutex> mutex = ptr->mutex;

		mutex->lock();
		if( !unique() )
			ptr = std::make_shared<Content>( ptr->obj->clone() );	// create a copy of the content
		mutex->unlock();
	}
}
*/

template<typename T>
void
shared_unique_ptr<T>::make_unique() {
	if( ptr ) {
		std::mutex *mutex = &ptr->mutex;

		mutex->lock();
		if( !unique() )
			ptr = std::make_shared<Content>( *ptr );	// create a copy of the content
		mutex->unlock();
	}
}

template<typename T>
void
shared_unique_ptr<T>::make_unique_clone() {
	if( ptr ) {
		std::mutex *mutex = &ptr->mutex;

		mutex->lock();
		if( !unique() )
			ptr = std::make_shared<Content>( ptr->obj->clone() );	// create a copy of the content
		mutex->unlock();
	}
}

} /* namespace efs */
