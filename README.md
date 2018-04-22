# hzh_slam
**Authors**: Zhenghua.Hou HIT 16S108281

**2017.2.12**: Finish this stereo VO.

**Introduce**： This is a very simple VO that can only be run on KITTI stereo datasets. It has not yet provided a visualization and an interface to save camera trajectory. This VO is a refactoring of **ORB_SLAM2**. The entire framework and the front-end are modeled after it. In contrast, this VO simplifies the back-end LocalMapping thread and simplifies the neighboring frame relationship of the Covisibility Graph.

**References**: [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) by Raulmur.

# 1. License
Only Myself and My junior of laboratory.

# 2. Prerequisites
## C++11 or C++0x Compiler
I use the new thread and chrono functionalities of C++11.
## OpenCV
I use [OpenCV](http://opencv.org).
## PCL
I use [PCL](http://pointclouds.org).
## Eigen
I use [Eigen](http://eigen.tuxfamily.org).
## g2o (Included in Thirdparty folder)  
I use [g2o](https://github.com/RainerKuemmerle/g2o).  

# 3. Building 
```
cd hzh_slam/Thirdparty/g2o
mkdir build
cd build 
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../../
mkdir build
cd build 
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```
# 4. Usage  
## KITTI Dataset
1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php
2. Execute the following command. Change KITTIX. yamlby KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change PATH_TO_DATASET_FOLDER to the uncompressed dataset folder. Change SEQUENCE_NUMBER to 00, 01, 02,.., 11.
```
./Examples/hzhslam_kitti Examples/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

