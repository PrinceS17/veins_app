# MyVeinsApp for Space-Air-Ground Vehicular Network Framework

## Overview
### 1, Background of AVE & Veins
Autonomous Vehicular Edge (AVE) is a framework for edge computing on the road to increase the computational capabilities of vehicles. \[1\] 

### 2, About the Softwares
The framework works on both Linux and Windows systems and it is mainly built and tested on ubuntu 16.04 LTS. SUMO 0.30.0, OMNeT++ 5.1.1, Veins 4.6 are used for simulation. For details about installation of Veins, please refer to [Veins tutorial][1]. You can also get enough information of SUMO and OMNeT++ there. 

[1]:http://veins.car2x.org/tutorial/

## Data Structures
### 0. Wave Short Message
### 1. struct *job*
### 2. queue\<int\> *job_queue*
### 3. vector\<int\> *job_vector*

## Stages
### 1. Beaconing
#### 1) Phase in processor: send beacons

#### 2) Phase in requester: receive beacons

### 2. Job Caching
* Phase in requester: generate jobs

### 3. Discovery
#### 1) Phase 0 in requester: request information

#### 2) Phase in processor: make response

#### 3) Phase 1 in requester: collect information

### 4. Scehduling

### 5. Data Transmission
#### 1) Phase 0 in requester: send job brief & data

#### 2) Phase in processor: receive & process job

#### 3) Phase 1 in requester: get result

## References
\[1\]: J. Feng, Z. Liu, C. Wu and Y. Ji, "[AVE: Autonomous Vehicular Edge Computing Framework with ACO-Based Scheduling][2]," in IEEE Transactions on Vehicular Technology, vol. 66, no. 12, pp. 10660-10675, Dec. 2017.

[2]: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7946184&isnumber=8207705
