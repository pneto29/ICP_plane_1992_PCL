# IterativeClosestPoint (point to plane metric) #

This repository contains an implementation of the Iterative closest point. 

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.

* PCL (Point cloud library) - 1.7 version
* Linux Ubuntu 16.04 or 18.04
* Qt Creator 4.5.2

## Usage ##

* Create a folder named build (for example)
mkdir build
* Enter folder
cd build/
* Run "CmakeLists.txt" which is a path before build
cmake ..
* Run make
make
* When the program has been built, run: 
./icp_p2pt argv[1] argv[2] argv[3]

## Parameters ##
* argv[1]: target cloud
* argv[2]: source cloud
* argv[3]: k number of neighbors
* argv[4]: cloud final


## Reference ##

Chen, Yang, and Gérard Medioni. "Object modelling by registration of multiple range images." 
Image and vision computing 10.3 (1992): 145-155.
