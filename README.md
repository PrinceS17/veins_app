# MyVeinsApp for Space-Air-Ground Vehicular Network Framework

## Overview
### 1, Background of the Framework
Our Space-Air-Ground (SAG) vehicular network framework focuses on mobile edge computing among vehicles, road side units (RSU) and some aerial communication platforms like unmanned aerial vehicles (UAV). The vehicle in the framework offloads its tasks to other nodes with computing capability. Therefore, the total computational delay is reduced and the computing resources are highly utilized. 

Autonomous Vehicular Edge (AVE) is a framework for edge computing on the road to increase the computational capabilities of vehicles. \[1\] The author provides a solution with ACO (Ant Colony Optimization) algorithm for the task offloading problem in dynamic environment among decentralized vehicles, which can allocation the computing resources more efficiently. Considering the reasonableness of the framework for practical vehicular network, we mainly realize our framework based on it with some modifications. 

One important difference is the assumption we make that one vehicle must be either requester (generate jobs and request computing resources) or processor (process jobs) , unlike that of AVE framework where every vehicle is both requester and processor. We suppose that only specific vehicles and platforms can provide computing resources in the near future. This difference causes a few changes of the stage, which will be explained in the following parts. 

### 2, About the Softwares
The framework is based on Veins, which is simulator for vehicular network. It combines SUMO, a traffic simulation platform, and OMNeT++, an IDE for network simulation and realizes a great amont of communication elements such as IEEE 802.11p, WAVE and two-ray interference channel, which makes it convenient to build a vehicular network framework and analyze the simulation results. \[2\]

The framework works on both Linux and Windows systems and it is mainly built and tested on ubuntu 16.04 LTS. SUMO 0.30.0, OMNeT++ 5.1.1, Veins 4.6 are used for simulation. For details about installation of Veins, please refer to [Veins tutorial][1]. You can also get enough information of SUMO and OMNeT++ there. 

[1]:http://veins.car2x.org/tutorial/

## Data Structures
### 1. Messages
WaveShortMessage(WSM), WaveServiceAdvertisement(WSA), BasicSafetyMessage (BSA) 


### 2. Jobs
struct *job*, queue\<job\> *job_queue*vector\<job\> *job_vector*

### 3. NAI (Neighbor Availability Index) Table
NAI entry, NAI table

### 4. Work
map\<int, job\> *work_info*

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
\[1\]  J. Feng, Z. Liu, C. Wu and Y. Ji, "[AVE: Autonomous Vehicular Edge Computing Framework with ACO-Based Scheduling][2]," in IEEE Transactions on Vehicular Technology, vol. 66, no. 12, pp. 10660-10675, Dec. 2017.

[2]: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7946184&isnumber=8207705

\[2\] Christoph Sommer, Reinhard German and Falko Dressler, "[Bidirectionally Coupled Network and Road Traffic Simulation for Improved IVC Analysis][3]," IEEE Transactions on Mobile Computing, vol. 10 (1), pp. 3-15, January 2011.

[3]: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5510240&isnumber=5640589
