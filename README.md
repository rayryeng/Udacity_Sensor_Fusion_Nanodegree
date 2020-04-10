# Udacity Sensor Fusion Nanodegree Course

This contains my homework assignments and quiz solutions for the programming
portions of this nanodegree. They will be partitioned into separate directories, 
one for each course. Each course will have separate build instructions.

# Instructions for building and running

## Lidar Obstacle Detection Course

This can be found in the `SFND_Lidar_Obstacle_Detection` directory in this repo.
This was initially a fork from https://github.com/udacity/SFND_Lidar_Obstacle_Detection,
but I made significant changes to the formatting, fixed some bugs that were seen on the
Issues page of the original repo and diverged away a bit from the original specifications
for submitting the project. However, the directory essentially contains the completed
project and does fulfill the guidelines required to gain a successful pass.

Please note that it is assumed that you have all of the dependencies installed for
this to build successfully. For instructions on getting these dependencies, please
refer to the original fork from the link above. However, you should only need to
install PCL as the extra dependency, which will tie in all of the other dependencies
required upon installation (Boost in particular).

To build this project, there are three submodules.

### Main environment

The main environment for the detection can be built navigating to the
`SFND_Lidar_Obstacle_Detection` directory and using the `CMakeLists.txt` file in
that directory:

``` 
$ cd SFND_Lidar_Obstacle_Detection
$ mkdir build && cd build
$ cmake ..
$ make
```

You can then run the environment by just doing:

``` 
$ ./environment
```

### RANSAC quiz

The portion of the course that tests the implementation of 3D plane estimation using RANSAC
can be built navigating to the `SFND_Lidar_Obstacle_Detection/src/quiz/ransac` directory
and using the `CMakeLists.txt` file in that directory:

```
$ cd SFND_Lidar_Obstacle_Detection/src/quiz/ransac
$ mkdir build && cd build
$ cmake ..
$ make
```

You can then run the clustering quiz by:

```
$ ./quizRansac
```

### Cluster quiz

The portion of the course that tests the implementation of Euclidean clustering can be
built navigating to the `SFND_Lidar_Obstacle_Detection/src/quiz/cluster` directory and
using the `CMakeLists.txt` file in that directory:

```
$ cd SFND_Lidar_Obstacle_Detection/src/quiz/cluster
$ mkdir build && cd build
$ cmake ..
$ make
```

You can then run the clustering quiz by:

```
$ ./quizCluster
```