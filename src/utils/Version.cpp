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

#include "efslam/utils/Version.h"
#include "VersionInfo.h"

namespace efs {

std::string
Version::name() {
	return "Evidential FastSLAM";
}


std::string
Version::description() {
	return "An evidential approach to SLAM";
}


std::string
Version::nameDescription() {
	return name() + " - " + description();
}


std::string
Version::author() {
	return "Joachim Clemens";
}


std::string
Version::email() {
	return "jaycee@informatik.uni-bremen.de";
}


std::string
Version::coauthors() {
	return "Thomas Reineking, Tobias Kluth";
}


std::string
Version::contact() {
	return author() + " <" + email() + ">";
}


std::string
Version::company() {
	return "University of Bremen";
}


std::string
Version::department() {
	return "Cognitive Neuroinformatics";
}


std::string
Version::copyright() {
	return "Copyright (c) 2013-2016, " + author() + ", " + coauthors();
}


std::string
Version::version() {
  return versionStr;
}


std::string
Version::buildDate() {
  return buildDateStr;
}


std::string
Version::commitHash() {
  return commitHashStr;
}


std::string
Version::commitDate() {
  return commitDateStr;
}


std::string
Version::branchName() {
  return branchNameStr;
}


std::string
Version::info() {
	std::string res;

	for( auto &line : infoLines() )
		res += line;

	return res;
}


std::vector<std::string>
Version::infoLines() {
	std::vector<std::string> res;

	res.push_back( nameDescription() );
	res.push_back( "Version " + version() + " from " + commitDate() + " build on " + buildDate() );
	res.push_back( copyright() );
	res.push_back( company() + ", " + department() );
	res.push_back( "Contact: " + contact() );

	return res;
}

} /* namespace efs */
