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

#ifndef EFS_RAWPTR_H_
#define EFS_RAWPTR_H_

namespace efs {

template<typename T>
class raw_ptr {
public:
	inline						raw_ptr() : ptr( nullptr ) {}
	inline						raw_ptr( T *p ) : ptr( p ) {};
	inline						raw_ptr( const raw_ptr &other ) : ptr( other.ptr ) {}
	inline						raw_ptr( raw_ptr &&other ) : ptr( std::move( other.ptr ) ) {}
	inline						~raw_ptr() {};

	inline	raw_ptr&			operator=( const raw_ptr &other ) 		{ if( this != &other ) { ptr = other.ptr; }; return *this; }
	inline	raw_ptr&			operator=( raw_ptr &&other )			{ ptr = std::move( other.ptr ); return *this; }

	inline	T&					operator*() noexcept			{ return *ptr;	}
	inline	T*					operator->() noexcept			{ return ptr;	}
	inline	T*					get() noexcept					{ return ptr;	}

	inline	const T&			operator*() const noexcept		{ return *ptr;	}
	inline	const T*			operator->() const noexcept		{ return ptr;	}
	inline	const T*			get() const noexcept			{ return ptr;	}

	inline						operator bool() const noexcept 	{ return (bool) ptr;	}

private:
	T	*ptr;
};

} /* namespace efs */

#endif /* EFS_RAWPTR_H_ */
