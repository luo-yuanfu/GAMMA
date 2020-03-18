# GAMMA: A General Agent Motion Prediction Model for Autonomous Driving

## Overview
GAMMA is a general agent motion prediction model for autonomous driving, implementing the code in the paper:

Luo, Y., Cai, P., Hsu, D., and Lee, W.S. GAMMA: A General Agent Motion Prediction Model for Autonomous Driving. arXiv preprint arXiv:1906.01566 (2019).

## Requirements

Tested Operating System: Linux 16.04.

Tested Compilers: gcc | g++ 5.4.0 or above

Tested Hardware: Intel Core i7 CPU, 8.0 GB RAM

## Download

Clone the repository from Github (**Recommended**):
```bash
$ git clone https://github.com/AdaCompNUS/GAMMA.git
```
OR manually download the [Zip Files](https://github.com/AdaCompNUS/GAMMA/archive/master.zip). For instructions, use this online Github README. 

## Installation

Follow following steps to compile:
```bash
$ cd GAMMA
$ mkdir build -p && cd build
$ cmake ..
$ make
```

## GAMMA Prediction on Real-world Datasets

This repo provides GAMMA prediction on four datasets: ETH [1], UCY [2], UTWON, and CROSS. The latter two are datasets collected by our own, and contain various types of traffic agents. The datasets can be found in `/GAMMA/dataset` folder.

[1] Pellegrini, S., Ess, A., and Van Gool, L.. Improving data association by joint modeling of pedestrian trajectories and groupings. In European conference on computer vision (pp. 452-465). Springer, Berlin, Heidelberg. 2010.

[2] Leal-Taixé, L., Fenzi, M., Kuznetsova, A., Rosenhahn, B., & Savarese, S.. Learning an image-based motion context for multiple people tracking. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (pp. 3542-3549). 2014


To test the prediction accuracy of GAMMA for Eth dataset, simply run the following commands after compilation: 
```bash
$ cd GAMMA/build
$ ./examples/predictor
```
This will generate a `gamma_output.txt` file in `GAMMA/` containing the predicted motions and the ground truth. Run the following commands to compute the prediction accuracy:
```bash
$ cd GAMMA
$ python gamma_compute_accuracy.py
```

To test the prediction accuracy for other datasets, modify the following line in `GAMMA/examples/prediction/AgentInfo.cpp` file.
```
const DatasetNs::Dataset dataset = DatasetNs::Eth;
```
The candidate datasets include:
```
//ETH:
DatasetNs::Eth
DatasetNs::Hotel
//UCY:
DatasetNs::Zara01
DatasetNs::Zara02
DatasetNs::Univ001
DatasetNs::Univ003
//UTOWN:
DatasetNs::Utown
//CROSS:
DatasetNs::Cross_ct
```

