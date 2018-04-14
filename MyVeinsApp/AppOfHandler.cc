//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include <string>
#include <cstring>
//#include <sstream>
//#include <vector>
//#include <algorithm>
#include <random>
#include "AppOfHandler.h"
using namespace std;

// typedef void (*fType)(WaveShortMessage *);                      // def fType for handler map

// kind for self-message
#define SEND_DATA_EVT 2
#define SEND_MY_BC_EVT 3
#define GENERATE_JOB_EVT 4
#define CHECK_EREQ_EVT 5
#define ENTER_SCH_EVT 6

// kind for received message
#define onB 7       // R & P: my beacon
#define onQ 8       // P: EREQ
#define onP 9       // R: EREP
#define onJ 10      // P: job brief
#define onD 11      // R & P: data
#define onT 12      // R & P: traffic info
#define sd 13

Define_Module(AppOfHandler);

// job parameters
double job_time = 0;
double lambda = 0.2;
double dataSize = 100e3;
double resultSize = 100e3;
double work_mean = 4;
double slot = 0.001;         // what is the typical value? 

bool TxEnd;

double rtd = 0.05;           // 50 ms
double t_disc = 0;
double speedLimit = 15;      // 15 m/s
double my_bc_interval = 1;   // 1 s, same as the BSM period
double ereq_interval = 0.02; // compared to rtd

bool ifFirst = true;
double t_end;                // for end of caching

// global variable
simtime_t job_delay;

// Poission Process Setting
default_random_engine job_g, work_g;
exponential_distribution<double> jobInterval_dstrb(lambda);         // intervals between jobs
exponential_distribution<double> workload_dstrb(1 / work_mean);     // job workload

void formal_out(const char*, int);

bool speed_cmp(double vx, double vy, double vz, Coord curSpeed)     // speed appropriate: true; too large: false
{
    Coord sourceSpeed(vx,vy,vz);
    if(curSpeed.distance(sourceSpeed) > speedLimit)                 // compare the speed
    { 
        formal_out("Speed difference too large!", 3);
        return false;
    }
    return true;
}

void AppOfHandler::relay(WaveShortMessage* wsm)
{
    wsm->setSenderAddress(myId);
    wsm->setSerial(wsm->getSerial() +1);
    sendDown(wsm->dup());
    string str = "Relay! Current serial is " + to_string(wsm->getSerial());
    formal_out(str.c_str(), 3);
}

bool AppOfHandler::on_data_check(WaveShortMessage* wsm, int srcId)  // not pass: return false
{
    if(srcId == myId)                               // if receiving my message, must be out of date
    {   
        formal_out("Discard self message!", 1); 
        return false; 
    }
    if(wsm->getRecipientAddress() != myId)                               // message not for me
    {   
        relay(wsm); 
        return false;
    }
    return true;
}

bool AppOfHandler::checkWSM(WaveShortMessage* wsm)
{
    if(strlen(wsm->getWsmData()) < 50)
        formal_out(wsm->getWsmData(), 1);
    else
    {
        string temp;
        temp.assign(wsm->getWsmData(), wsm->getWsmData() + 50);
        formal_out((temp + "... for short").c_str(), 1);
    }

    if(current_state <= ST) return false;           // if still initializing
    if(wsm->getSerial() >= 3)                       // discard 3-hop message or self message
    {
        formal_out("Discard 3-hop message!", 1);
        return false;
    }

    if(NULL == wsm->getWsmData()) {formal_out("Error: void pointer!", 1); return false;}              // check if void pointer
    if(strlen(wsm->getWsmData()) <= 1) {formal_out("Error: receive too little data!", 1); return false;}       // check data length   
    return true;
}

vector<int> AppOfHandler::scheduling(vector<int> v0, int type)
{
    current_state = SCH;
    int job_size = v0.at(0), max_size = v0.at(1);
    formal_out("Entering scheduling phase!", 3);
    vector<job> temp(job_vector.begin() + job_size, job_vector.begin() + job_size + max_size);
    vector<int> serviceCar;
    for(int i = 0; i < temp.size(); i ++)
    {
        map<int, double> bid = temp[i].bid;
        vector<pair<int, double>> bid_vec(bid.begin(), bid.end());
        int ri;
        switch(type)
        {
        case 0:             // choose randomly
            if(bid.size()) ri = rand() % static_cast<int>(bid.size());     // cause error code 136 if bid.size() = 0
            else {formal_out("Error: bid size is 0!\n\n\n", 1); exit(0);}
            // ri = 10000 % static_cast<int>(bid.size());                  // for debug, eliminate the probability
            serviceCar.push_back(bid_vec[ri].first);      
            break;
        case 1:             // choose the vehicle who has the least bid
            sort(bid_vec.begin(), bid_vec.end(), cmp);
            serviceCar.push_back(bid_vec[0].first);
        default: ;
        }
    }  
    serviceCar.emplace(serviceCar.begin(), job_size);
    return serviceCar; 
}

void AppOfHandler::send_data(int size, int rcvId, int serial, simtime_t time)        // send data of size wave short messages of 4k except the last wsm
{
    formal_out("sending data...", 3);
    int max_size = 4096;                    // max size of wsm data, except "D" at the start
    int num = (size + 2)/max_size + 1;      // number of wsm, 2 is "ed"
    int last_size = (size + 2)% max_size;   // last wsm size
    for(int i = 0; i < num; i ++)
    {
        int l = max_size;
        if(i == num - 1) l = last_size;
        stringstream ss;
        ss << myId <<' ';
        for(int j = ss.str().size(); j < l; j ++) ss<< '0';
        if(i == num - 1) ss<< "ed";

        WaveShortMessage * data = new WaveShortMessage();
        populateWSM(data, rcvId, serial);
        data->setKind(onD);
        data->setWsmLength(l);
        data->setWsmData(ss.str().c_str());  
        sendDown(data);
        if(current_task_time <= simTime()) idleState = true;
    }
}

void AppOfHandler::send_data(job myJob, int rcvId, int serial, simtime_t time)       // overload for requester sending data with job brief
{   
    formal_out("sending job brief...", 3);   
    stringstream ss;
    ss<< myId <<' '<< myJob.data_size <<' '<< myJob.result_size <<' '<< myJob.workload <<' '<< myJob.utility <<' '<< myJob.bid[rcvId] <<' '<<myJob.start;
    WaveShortMessage * pre_data = new WaveShortMessage();
    populateWSM(pre_data, rcvId, serial);                       // job brief sent before real data
    pre_data->setKind(onJ);
    pre_data->setWsmData(ss.str().c_str());
    sendDown(pre_data);   
    send_data(myJob.data_size, rcvId, serial, time + slot);     // send data of job's data sizes
}

void AppOfHandler::local_process(queue<job> job_queue)
{
    formal_out("locally process...", 3);
    int max_size = job_queue.size();
    int job_size = job_vector.size();
    for(int i = 0; i < max_size; i ++)
    {
        job myJob = job_queue.front();
        job_queue.pop();
        if(current_task_time > simTime())
            current_task_time += (double) myJob.workload / computing_speed;
        else
        {
            idleState = false;
            current_task_time = simTime() + (double) myJob.workload / computing_speed;
        }    
        myJob.delay = current_task_time - myJob.start;          // recored the final delay
        job_delay = myJob.delay;
        emit(sig, job_delay);                                    // use signal to record the delay of each job
        // delayVec.record(job_delay);  
    }    
}

void AppOfHandler::local_process(vector<job> job_vec)
{
    formal_out("locally process for EREQ...", 3);
    for(auto myJob:job_vec)
    {
        if(current_task_time > simTime())
            current_task_time += (double) myJob.workload / computing_speed;
        else
        {
            idleState = false;
            current_task_time = simTime() + (double) myJob.workload / computing_speed;
        }
        myJob.delay = current_task_time - myJob.start;          // recored the final delay
        job_delay = myJob.delay;
        emit(sig, job_delay);         
    }
}


void AppOfHandler::handleBeacon(WaveShortMessage* wsm)
{
    if(current_state == ST) current_state = BEA;
    string nodeState = node_type ? "Requester: my beacon...":"Processor: my beacon...";
    formal_out(nodeState.c_str(), 2);

    stringstream ss(wsm->getWsmData());
    int vehicleId;
    double vx, vy, vz;
    bool ifIdle;
    ss>> vehicleId >> vx >> vy >> vz >> ifIdle;
    simtime_t expiredTime = simTime() + my_bc_interval;
    if(!speed_cmp(vx, vy, vz, curSpeed)) return;
    vector<int> hop1_neighbor;
    while(ss.str().length() - ss.tellg() > 1)               // decode neighbor ID and update the NAI
    {
        int neighborId;
        ss>> neighborId;

        int hopNum;
        if(!naiTable.find(neighborId))                      // if it's a new entry: create
        {
            if(neighborId == vehicleId)                     // if this entry is the source
                hopNum = 1;
            else
                hopNum = 2;
            naiTable.push_back(neighborId, ifIdle, hopNum, expiredTime);
        }
        else if(neighborId == vehicleId)                    // if the source entry is already in the table: update
        {
            if(naiTable.NAI_map[vehicleId].hopNum == 2 )
            {
                naiTable.NAI_map[vehicleId].hopNum = 1;
                naiTable.NAI_map[vehicleId].expiredTime = expiredTime;
            }
            else naiTable.NAI_map[vehicleId].expiredTime = expiredTime;
        }
        else ;                                              // if other entry is in the table: no change
    }  
}

void AppOfHandler::sendDup(WaveShortMessage* wsm)
{
    wsm->setKind(onB);          // only for debug, to send beacon
    sendDown(wsm->dup());    
}

void AppOfHandler::handleTraffic(WaveShortMessage* wsm)
{
    formal_out("traffic info...", 2);
    const char* wdata = wsm->getWsmData();
    findHost()->getDisplayString().updateWith("r=16,green");
    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wdata, 9999);
    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setKind(onT);
        wsm->setSerial(3);
        sendDown(wsm->dup());       // ? no wait time now
    }
}

void AppOfHandler::generateJob(WaveShortMessage* wsm)
{
    if(current_state == ST) current_state = CAC;
    string str = node_type == REQUESTER? "Requester":"Processor";
    str += ": generate jobs..."; 
    formal_out(str.c_str(), 2);                             // for debug
    double workload = workload_dstrb(work_g);
    job_time += workload;

    job myJob = {dataSize, resultSize, workload};           // generate and push job to queue
    myJob.start = simTime().dbl();
    job_queue.push(myJob);
    str = "queue length: " + to_string(job_queue.size()) + " ; last job: " + to_string(job_queue.back().workload);
    formal_out(str.c_str(), 3);    

    WaveShortMessage * jobMsg = new WaveShortMessage();     // send wsm to generate next job
    populateWSM(jobMsg);
    jobMsg->setKind(GENERATE_JOB_EVT);
    double x = jobInterval_dstrb(job_g);
    scheduleAt(simTime() + x, jobMsg);
}

void AppOfHandler::handleCache(WaveShortMessage* wsm)
{
    naiTable.update();
    double Q = current_task_time.dbl() - simTime().dbl();
    Q = Q < 0? 0:Q;
    string str = "NAI value: " + to_string(naiTable.value) + " ; Q: " + to_string(Q);
    formal_out(str.c_str(), 2);
    if(ifFirst)
    {
        t_end = simTime().dbl() + Q / (naiTable.value + 1);
        ifFirst = false;
    }
    if(!naiTable.value && !Q)              // if idle and no processor: process its own jobs
    {
        local_process(job_queue);          // combination of send EREQ and dat
        ifFirst = true;
    }
    else if(TxEnd && (job_queue.size() > naiTable.value + 1 || simTime() > t_end))
    {
        handleDISQ(wsm);
        t_disc = simTime().dbl();          // update the begin time of discovery
        ifFirst = true;
    } 
    generateJob(wsm);
}

void AppOfHandler::handleDISQ(WaveShortMessage* wsm)
{
    vector<int> v0;
    if(current_state < DISQ) current_state = DISQ;
    // naiTable.update();
    // send_EREQ(job_queue, job_time);     // queue popped in send_EREQ
    formal_out("sending EREQ...", 3);

    stringstream EREQ;
    EREQ<< myId <<' '<< curSpeed.x <<' '<< curSpeed.y <<' '<< curSpeed.z <<' ';      
    int max_size = job_queue.size();             // set the index of job segment
    int job_size = job_vector.size();
    EREQ<< job_size <<' '<< max_size<<' ';       // identify the position when coming back   
    for(int i = 0; i < max_size; i ++)           // write data to EREQ
    {
        job temp = job_queue.front();
        temp.utility = temp.workload / (temp.workload + job_time);
        job_time -= temp.workload;               // used above to calculate utility
        job_vector.push_back(temp);
        EREQ<< temp.workload <<' ';
        job_queue.pop();
    }
    WaveShortMessage * myEREQ = new WaveShortMessage();     // send EREQ
    populateWSM(myEREQ);
    myEREQ->setKind(onQ);
    myEREQ->setWsmData(EREQ.str().c_str());
    sendDown(myEREQ->dup());
    myEREQ->setKind(CHECK_EREQ_EVT);                        // check if EREQ needs to be resent
    scheduleAt(simTime() + ereq_interval, myEREQ);       
    job_time = 0;
    TxEnd = false;
}

void AppOfHandler::checkEREQ(WaveShortMessage* wsm)
{
    string temp = current_state == DISP? "DISP": (current_state == DISQ? "DISQ":"not DIS: " + to_string(current_state));
    string str = "check EREQ ... current state is " + temp + "; ddl: " + to_string(t_disc + rtd);
    formal_out(str.c_str(), 2);
    if(current_state == DISQ)
    {
        if(simTime() + ereq_interval < t_disc + rtd)                    // another check for EREQ if time allows     
        {
            scheduleAt(simTime() + ereq_interval, wsm->dup());
            wsm->setKind(0);
            sendDown(wsm->dup());                                           // send EREQ before t_disc+rtd
        }

        else 
        {                                                                   // read job_size and max_size from EREQ
            int vehicleId, job_size, max_size;
            double vx, vy, vz;
            stringstream ss(wsm->getWsmData());
            ss >> vehicleId >> vx >> vy >> vz >> job_size >> max_size;       
            vector<job> job2proc(job_vector.begin() + job_size, job_vector.begin() + job_size + max_size);
            local_process(job2proc);
        }
    }
}

void AppOfHandler::handleDISP(WaveShortMessage* wsm)
{
    int job_size = -1, max_size = -1, vehicleId;
    vector<int> v0;
    formal_out("EREP...", 2);
    stringstream ss(wsm->getWsmData());
    ss>> vehicleId >> job_size >> max_size; 
    if(naiTable.find(wsm->getRecipientAddress()) && naiTable.NAI_map[wsm->getRecipientAddress()].hopNum == 1)       // relay
    {   formal_out("Forward EREP...", 3); relay(wsm); return; }
    if(wsm->getRecipientAddress() != myId) return;                         // message for other nodes

    current_state = DISP;            
    formal_out("Get bid information...", 3);                               // process my EREP now
    for(int i = job_size; i < job_size + max_size; i ++)                   // obtain correct position
    {
        double bid;
        ss >> bid;
        job_vector[i].bid[vehicleId] = bid;
    }       
    v0.push_back(job_size);
    v0.push_back(max_size);
    if(v0.at(0) < 0 || v0.at(1) < 0) { formal_out("Error: no job size, max size stored in vector v0!", 1); exit(0);}

    WaveShortMessage * temp = new WaveShortMessage();
    temp->setKind(ENTER_SCH_EVT);
    ss << v0[0] << " " << v0[1] << endl;            // carry v0 vector
    temp->setWsmData(ss.str().c_str());
    if(simTime().dbl() - t_disc >= rtd)  
        scheduleAt(simTime(), temp);                // if time expired, enter SCH/DAT state
    else scheduleAt(t_disc + rtd, temp);            // wait (what if many EREPs handled & scheduled?)
}

void AppOfHandler::sendData(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    vector<int> v0(2,0);
    ss >> v0.at(0) >> v0.at(1);
    vector<int> serviceCar = scheduling(v0, 0);         // SCH state: may take some time

    current_state = DAT;                                // DAT state
    idleState = false;
    int job_size = serviceCar[0];
    serviceCar.erase(serviceCar.begin());
    for(int i = 0; i < serviceCar.size(); i ++)
    {   
        send_data(job_vector[job_size + i], serviceCar[i], 0, simTime());           
        work_info[serviceCar[i]] = job_vector[job_size + i];
    }
    idleState = true;   
    TxEnd = true;
    current_state = CAC;
}

void AppOfHandler::handleResultData(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    formal_out("data received...", 2);
    int vehicleId;                                  // Id of the requester
    ss>> vehicleId;
    if(!on_data_check(wsm, vehicleId)) 
        return;
    job myJob = work_info[vehicleId];               // get info of current job 
    ss.seekg(-2, ss.end);
    string temp; 
    ss>> temp;
    if(temp == "ed")
    {
        myJob.delay = simTime() - myJob.start;
        job_delay = myJob.delay;
        string str = "Finish receving result from " + to_string(vehicleId) + " ; delay = " + to_string(job_delay.dbl());
        formal_out(str.c_str(), 1);
        emit(sig, job_delay);                       // use signal to record the delay of each job 
        work_info.erase(vehicleId);
    }
}


void AppOfHandler::sendBeacon(WaveShortMessage* wsm)
{
    if(current_state == ST) current_state = BEA;
    formal_out("Processor: send beacon...", 2);
    naiTable.update();
    vector<int> hop1_Neighbor = naiTable.generate_hop1();
    stringstream ss;
    ss<< myId <<' '<< curSpeed.x <<' '<< curSpeed.y <<' '<< curSpeed.z <<' '<< idleState <<' ';
    for(int a:hop1_Neighbor) ss << a << ' ';
    WaveShortMessage * bc = new WaveShortMessage();
    populateWSM(bc);
    bc->setWsmData(ss.str().c_str());               // set current beacon
    
//    bc->setKind(onB);
//    sendDown(bc->dup());
    bc->setKind(sd);
    scheduleAt(simTime(), bc->dup());
    
    bc->setKind(SEND_MY_BC_EVT);                    // schecule next beacon
    scheduleAt(simTime() + my_bc_interval, bc->dup());
}

void AppOfHandler::processEREQ(WaveShortMessage* wsm)
{
    int job_size = -1, max_size = -1, vehicleId;
    double vx, vy, vz;
    vector<int> v0;
    if(current_state < DISQ) current_state = DISQ;
    formal_out("EREQ...", 2);
    stringstream ss(wsm->getWsmData());
    ss>> vehicleId >> vx >> vy >> vz;
    if(!speed_cmp(vx, vy, vz, curSpeed)) return;
    if(naiTable.find(vehicleId) && naiTable.NAI_map[vehicleId].hopNum == 1)          // 1 hop node forward EREQ
    {   formal_out("Forward EREQ...", 3); relay(wsm); return; }

    bool compatibility = rand() < 0.75*RAND_MAX? true:false;                         // consider compatibility
    if(!compatibility) formal_out("Not compatible!", 3);
    else
    {
        string str = "sending EREP to " + to_string(vehicleId) + "...";
        formal_out(str.c_str(), 3);

        stringstream EREP;
        int nump = 0, job_size, max_size;
        ss >> job_size >> max_size;
        EREP << myId <<' '<< job_size <<' '<< max_size <<' ';
        while(ss.str().length() - ss.tellg() > 1)                                // calculate bids for each job
        {
            double workload, bid;
            ss >> workload;
            bid = workload / computing_speed;
            EREP << bid <<' ';
            nump ++;
        }
        WaveShortMessage * myEREP = new WaveShortMessage();                          // send to vehicleId
        populateWSM(myEREP, vehicleId);
        myEREP->setKind(onP);
        myEREP->setWsmData(EREP.str().c_str());
        sendDown(myEREP);
    }
}

void AppOfHandler::processJobBrief(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    int vehicleId;
    ss>> vehicleId; 
    if(!on_data_check(wsm, vehicleId)) 
        return;
    if(current_state < DAT) current_state = DAT;       // ? right
    formal_out("job brief...", 2);            
    job myJob;
    ss>> myJob.data_size >> myJob.result_size >> myJob.workload >> myJob.utility >> myJob.bid[myId] >> myJob.start;        // index of bid doesn't matter
    work_info.insert(pair<int, job>(vehicleId, myJob));                                                                    // store source Id and the job in map!!!
}

void AppOfHandler::processData(WaveShortMessage* wsm)
{
    formal_out("data received...", 2);
    stringstream ss(wsm->getWsmData());
    int vehicleId;                                  // Id of the requester
    ss>> vehicleId;
    if(!on_data_check(wsm, vehicleId)) 
        return;
    if(current_state < DAT ) current_state = DAT;   // ? OK for process
    job myJob = work_info[vehicleId];               // get info of current job 
    ss.seekg(-2, ss.end);
    string temp; 
    ss>> temp;
    if(temp == "ed")   
    {
        if(current_task_time > simTime())
            current_task_time += (double) myJob.workload / computing_speed;           // here the computing speed can also be specified for different cars
        else
        {
            idleState = false;
            current_task_time = simTime() + (double)myJob.workload / computing_speed;
        }
        send_data(myJob.result_size, vehicleId, 0, current_task_time);                // send back result data
        work_info.erase(vehicleId);
    }
    current_state = BEA;
}


void AppOfHandler::initialize(int stage) {
    formal_out("Initializing...", 1);

    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << endl;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        // srand((unsigned)time(0));
        node_type = rand() < 0.5*RAND_MAX? REQUESTER:PROCESSOR;              // can be initialized by the position or other properties of nodes

        current_task_time = 0;
        computing_speed = uniform(1,2);                                      // can be specified by our demand
        idleState = true;
        TxEnd = true;
        naiTable.push_back(myId, idleState, 0, simTime() +  1000 * my_bc_interval);       // my ID should never expired? 200 correctly
        current_state = ST;
        sig = registerSignal("sig");

    }
    else if (stage == 1) {
        //Initializing members that require initialize other modules goes here
        WaveShortMessage* ini = new WaveShortMessage();
        if(node_type == PROCESSOR)
        {
            Handler[onB] = &AppOfHandler::handleBeacon;
            Handler[onT] = &AppOfHandler::handleTraffic;

            Handler[SEND_MY_BC_EVT] = &AppOfHandler::sendBeacon;  
            Handler[onQ] = &AppOfHandler::processEREQ;
            Handler[onJ] = &AppOfHandler::processJobBrief;
            Handler[onD] = &AppOfHandler::processData;
//            
            Handler[sd] = &AppOfHandler::sendDup;           // only for debug
// for unit test of beaconing
            
            sendBeacon(ini); 
        }
        else
        {
            Handler[onB] = &AppOfHandler::handleBeacon;
            Handler[onT] = &AppOfHandler::handleTraffic;

            Handler[GENERATE_JOB_EVT] = &AppOfHandler::handleCache;
            Handler[CHECK_EREQ_EVT] = &AppOfHandler::checkEREQ;
            Handler[onP] = &AppOfHandler::handleDISP;
            Handler[ENTER_SCH_EVT] = &AppOfHandler::sendData;
            Handler[onD] = &AppOfHandler::handleResultData;
//            
            generateJob(ini);              // virtual job, to starting normal job caching
            t_disc = simTime().dbl();
        }
    }
}


void AppOfHandler::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void AppOfHandler::onBSM(BasicSafetyMessage* bsm) {
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here

}

void AppOfHandler::onWSA(WaveServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

    // example in TraCIDemo means that change channel according to wsa and not use more information
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}


void AppOfHandler::onWSM(WaveShortMessage* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples, 

    if(!checkWSM(wsm)) return;
    if(Handler.find(wsm->getKind()) != Handler.end())       // cannot find => type not maching
        (this->*Handler[wsm->getKind()])(wsm);
}



void AppOfHandler::handleSelfMsg(cMessage* msg) {
    //this method is for self messages (mostly timers)
    //it is important to call the BaseWaveApplLayer function for BSM and WSM transmission

    formal_out("Handling self message...", 1);
    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) 
    {
        if(Handler.find(wsm->getKind()) != Handler.end())
            (this->*Handler[wsm->getKind()])(wsm);
        delete(wsm);        
    }
    else BaseWaveApplLayer::handleSelfMsg(msg);
}


void AppOfHandler::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    formal_out("Handling position update...", 1);
    findHost()->getDisplayString().updateWith("r=16,red");
    sentMessage = true;
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm);
    string tr_wsm = mobility->getRoadId();
    wsm->setWsmData(tr_wsm.c_str());

    //host is standing still due to crash
    if (dataOnSch) {
        startService(Channels::SCH2, 42, "Traffic Information Service");
        //started service and server advertising, schedule message to self to send later
        scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);         // might disabled by me
    }
    else {
        //send right away on CCH, because channel switching is disabled
        wsm->setKind(onT);
        sendDown(wsm);
    }
}

