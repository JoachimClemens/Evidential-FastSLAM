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

#ifndef EFS_SHAREDUNIQUEPTR_H_
#define EFS_SHAREDUNIQUEPTR_H_

#include <memory>
#include <mutex>

namespace efs {

/**
 * This class behaves like a normal std::shared_ptr, except for the fact
 * that make_unique() can be used in order to make it a unique copy
 * of the shared content. After that, it behaves like a std::shared_ptr
 * of the copied content.
 */
template<typename T>
class shared_unique_ptr {
public:
	inline						shared_unique_ptr();
	inline						shared_unique_ptr( T *p );
	inline						shared_unique_ptr( const shared_unique_ptr &other );
	inline						shared_unique_ptr( shared_unique_ptr &&other );
	inline						~shared_unique_ptr();

	inline	shared_unique_ptr&	operator=( const shared_unique_ptr &other );
	inline	shared_unique_ptr&	operator=( shared_unique_ptr &&other );

	inline 	void 				make_unique();
	inline 	void 				make_unique_clone();		// does the same as make_unique(), but calls the methid clone() (that has to be implemented) instead of the copy constructor
	inline 	bool				unique() const noexcept			{ return ptr.unique(); 						}
	inline	long int			use_count() const noexcept		{ return ptr.use_count(); 					}

	/*
	inline	void				lock()							{ ptr->mutex->lock();						}
	inline	bool				try_lock() noexcept				{ return ptr->mutex->try_lock();			}
	inline	void				unlock() noexcept				{ ptr->mutex->unlock();						}
	*/

	inline	void				lock()							{ ptr->mutex.lock();						}
	inline	bool				try_lock() noexcept				{ return ptr->mutex.try_lock();				}
	inline	void				unlock() noexcept				{ ptr->mutex.unlock();						}

	inline	T&					operator*() noexcept			{ return *ptr->obj;							}
	inline	T*					operator->() noexcept			{ return ptr->obj.get();					}
	inline	T*					get() noexcept					{ return ptr ? ptr->obj.get() : nullptr;	}

	inline	const T&			operator*() const noexcept		{ return *ptr->obj;							}
	inline	const T*			operator->() const noexcept		{ return ptr->obj.get();					}
	inline	const T*			get() const noexcept			{ return ptr ? ptr->obj.get() : nullptr;	}

	inline						operator bool() const noexcept 	{ return ptr && ptr->obj;					}

private:
	/*
	// Is this saver? Requires an additional allocation and additional space or shared_ptr counter.
	struct Content {
		inline			Content( T *p ) 				: mutex( new std::mutex ), obj( p ) {}
		inline			Content( const Content &other ) : mutex( new std::mutex ), obj( new T( *other.obj ) ) {}		// make a copy of the object
		inline			Content( Content &&other ) 		: mutex( std::move( other.mutex ) ), obj( std::move( other.obj ) ) {}

		// Assignment operator is not needed because it is not used

		std::shared_ptr<std::mutex>	mutex;
		std::unique_ptr<T>			obj;
	};
	*/

	struct Content {
		inline			Content( T *p ) 				: obj( p ) {}
		inline			Content( const Content &other ) : obj( new T( *other.obj ) ) {}		// make a copy of the object
		inline			Content( Content &&other ) 		: obj( std::move( other.obj ) ) {}

		// Assignment operator is not needed because it is not used

		std::mutex			mutex;
		std::unique_ptr<T>	obj;	// TODO: Some space can be saved (and one allocation?) by using an object instead of a pointer. However, this required an other construction of the shared_unique_ptr.
	};

	std::shared_ptr<Content>	ptr;
};

} /* namespace efs */

#include "shareduniqueptr.hpp"

#endif /* EFS_SHAREDUNIQUEPTR_H_ */
