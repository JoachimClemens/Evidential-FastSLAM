# Evidential FastSLAM
An evidential approach to Simultaneous Localization and Mapping

### Authors

Joachim Clemens, Thomas Reineking, Tobias Kluth

### Description 

The Evidential FastSLAM algorithm represents the world using a grid-map and utilizes a Rao-Blackwellized particle filter in order to approximate the joint distribution of path and map.
In contrast to other grid-map-based SLAM approaches, a belief function is used instead of a single occupancy probability to model the state of a grid cell.
It allows one to assign mass not only to the singletons of the hypotheses space, but also to all subsets.
As a consequence, the algorithm is able to express the uncertainty in the map more explicitly and one can distinguish between different uncertainty dimensions that are indistinguishable in a probabilistic grid map.
This additional information can be used for navigation tasks like path planning or active exploration.

### Paper Describing the Approach

Joachim Clemens, Thomas Reineking, Tobias Kluth, *An evidential approach to SLAM, path planning, and active exploration*, International Journal of Approximate Reasoning, in press, 2016, [doi:10.1016/j.ijar.2016.02.003](http://dx.doi.org/10.1016/j.ijar.2016.02.003).


### Example Maps

Resulting maps for the Intel Research Lab (raw dataset recorded by Dirk Hähnel) and Cartesium building, University of Bremen (raw dataset recorded by Cyrill Stachniss):

<image src="/images/intel.png" alt="Intel" height="200px" />
<image src="/images/cartesium.png" alt="Cartesium" height="200px" />

The color coding is red for occupied, green for free, blue for the superset (corresponding to unknown areas), and black for empty set (corresponding to conflicts).
The estimated path is shown in yellow.

### Software Requirements

The software is developed and tested on Ubuntu Linux 14.04.
It should run on other recent Linux and UNIX systems as well, while some modifications may be required to run it on Windows.
Furthermore, the following software is required:

* CMake (package `cmake`)
* Qt4 (package `libqt4-dev`)
* Eigen3 (package `libeigen3-dev`)

### Compilation and Running

Once all required software is installed, the code can be compiled and executed as follows:

```
git clone https://github.com/JoachimClemens/Evidential-FastSLAM.git
cd Evidential-FastSLAM
mkdir build
cd build
cmake ..
make
cd ../bin
./EFSlamCarmenGui --filename <carmen-log-file>
```

The input file has to be in CARMEN log format.
Be sure that you configure the sensor parameters with `--laser-start-angle` (default: -90.0) and `--laser-angular-res` (default: 1.0) correctly, which are both given in degrees.
Datasets are available e.g. at [Radish](http://cres.usc.edu/radishrepository/) or from [Cyrill Stachniss](http://www2.informatik.uni-freiburg.de/~stachnis/datasets.html), while one of the most popular datasets might be the one of the Intel Research Lab recorded by Dirk Hähnel (http://www2.informatik.uni-freiburg.de/~stachnis/datasets/datasets/intel-lab/intel.log.gz, see above).


### License Information

Evidential FastSLAM is published under the BSD License. See [LICENSE](LICENSE) for further information.
