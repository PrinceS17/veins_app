# MyVeinsApp for Space-Air-Ground Vehicular Network

## Overview

## Data Structure
### 0. Wave Short Message
### 1. struct *job*
### 2. queue\<int\> *job_queue*
### 3. vector\<int\> *job_vector*

## Stages
### 1. Beaconing
#### 1) Phase in processor: send beacons

#### 2) Phase in requester: receive beacons

### 2. Job Caching
#### * Phase in requester: generate jobs

### 3. Discovery
#### 1) Phase 0 in requester: request information

#### 2) Phase in processor: make response

#### 3) Phase 1 in requester: collect information

### 4. Scehduling

### 5. Data Transmission
#### 1) Phase 0 in requester: send job brief & data

#### 2) Phase in processor: receive & process job

#### 3) Phase 1 in requester: get result
