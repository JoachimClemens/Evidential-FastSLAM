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

#ifndef EFS_LOG_H_
#define EFS_LOG_H_


#ifndef NCOLOR
#	define BLACK 		"\033[30m"
#	define RED			"\033[31m"
#	define GREEN  		"\033[32m"
#	define YELLOW 		"\033[33m"
#	define BLUE   		"\033[34m"
#	define PINK   		"\033[35m"
#	define TURQ			"\033[36m"
#	define WHITE		"\033[37m"
#	define UNDERLINE	"\033[4m"
#	define DIM			"\033[2m]"
#	define BOLD 		"\033[1m"
#	define NORMAL 		"\033[0m"
#else
#	define BLACK		""
#	define RED			""
#	define GREEN  		""
#	define YELLOW 		""
#	define BLUE   		""
#	define PINK   		""
#	define TURQ			""
#	define WHITE		""
#	define UNDERLINE	""
#	define DIM			""
#	define BOLD			""
#	define NORMAL 		""
#endif

#include <iostream>
// no debug output if not in debug mode:
#ifdef NDEBUG
#	ifndef B3LSAM_NDEBUG_OUT
#		define B3LSAM_NDEBUG_OUT
#	endif
#endif
#ifdef B3LSAM_NDEBUG_OUT
#	define l_dbg( ... ) (void)0
#else
#	define l_dbg( args ) std::cout << BLUE BOLD "DEBUG:" << __FILE__ << "," << __LINE__ << NORMAL ": " << args << std::endl
#endif

#define l_inf( args ) std::cout << GREEN BOLD "INFO" NORMAL ":    " << args << std::endl
#define l_wrn( args ) std::cerr << YELLOW BOLD "WARNING" NORMAL ": " << args << std::endl
#define l_err( args ) std::cerr << RED BOLD "ERROR" NORMAL ":   " << args << std::endl
#define l_ftl( args ) std::cerr << RED BOLD "FATAL" NORMAL ":   " << args << std::endl


#define DEBUG_PRINT( x ) l_dbg( #x"=" << x )
#define DEBUG_PRINT_COUT( x ) std::cout << #x"=" << x << std::endl


#endif /* EFS_LOG_H_ */
