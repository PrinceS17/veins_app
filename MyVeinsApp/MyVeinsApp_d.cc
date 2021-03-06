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
#include <sstream>
#include <vector>
#include <algorithm>
#include <random>
#include "MyVeinsApp.h"
using namespace std;

#define SEND_DATA_EVT 2
#define SEND_MY_BC_EVT 3
#define GENERATE_JOB_EVT 4

Define_Module(MyVeinsApp);

/* here is AVE platform with each units encapsulated.
 * head of different wsm:
T: traffic msg of original code;
B: my beacon;
Q: EREQ at discovery;
P: EREP at discovery;
D: data at TX;
J: job brief before sending data
 */

// job parameters
double job_time = 0;
double lambda = 5;
double dataSize = 100e3;
double resultSize = 100e3;
double work_mean = 4;
double slot = 0.001;         // what is the typical value? 

bool TxEnd;

double rtd = 0.05;           // 50 ms
double t_disc = 0;
double speedLimit = 15;      // 15 m/s
double my_bc_interval = 1;   // 5 s

// global variable
simtime_t job_delay;
stringstream ss_null("");

// Poission Process Setting
default_random_engine job_g, work_g;
exponential_distribution<double> jobInterval_dstrb(lambda);         // intervals between jobs
exponential_distribution<double> workload_dstrb(1 / work_mean);      // job workload


void formal_out(const char * str, int lv)   // use output to debug
{
    switch(lv)
    {
    case 1:                     // event like onWSM
        EV<<" "<< str <<"\n";
        break;
    case 2:                     // option like send beacon
        EV<<"   "<< str <<"\n";
        break;
    case 3:                     // specific function like send EREQ
        EV<<"     "<< str <<"\n";
        break;
    default: ;
    }

}

void MyVeinsApp::bea(WaveShortMessage* wsm, stringstream* ss_ptr = &ss_null)
{
    
    stringstream ss(ss_ptr->str());
    if(ss.str() == "")          // processor sending beacon
    {
        formal_out("Processor: send beacon...", 2);
        naiTable.update();
        send_beacon(naiTable.generate_hop1());
    }
    else                    // beacon receiving by either processor or requester
    {
        formal_out("Requester: my beacon...", 2);

        int vehicleId;
        double vx, vy, vz;
        bool ifIdle;
        ss>> vehicleId >> vx >> vy >> vz >> ifIdle;
        simtime_t expiredTime = simTime() + my_bc_interval;

        // compare the speed
        Coord sourceSpeed(vx,vy,vz);
        if(curSpeed.distance(sourceSpeed) > speedLimit)
        {
            formal_out("Speed difference too large!", 3);
            // delete(wsm);
            return;
        }

        // decode the neighbor ID and update the NAI table
        vector<int> hop1_neighbor;

        while(!ss.eof())
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
}

void MyVeinsApp::cac()
{
    string str = "generate jobs... NAI value: " + to_string(naiTable.value + 1);
    formal_out(str.c_str(), 2);

    // generate a new job on this requester
    if(node_type == PROCESSOR) return;       
    // double workload = workload_dstrb(work_g);
    double workload = work_mean;
    job_time += workload;
    generate_job(lambda, dataSize, resultSize, workload);

}

vector<int> MyVeinsApp::dis(int phase, WaveShortMessage* wsm, stringstream* ss_ptr = &ss_null)
{
    stringstream ss(ss_ptr->str());
    int job_size = -1, max_size = -1;
    vector<int> v0;
    switch(node_type)
    {
    case REQUESTER:
        if(phase == 0)      // begin discovery by sending EREQ
        {
            naiTable.update();
            send_EREQ(job_queue, job_time);                             // queue popped in send_EREQ
            job_time = 0;
            TxEnd = false;
        }
        else                // end discovery by storing the info from EREP
        {
            formal_out("EREP...", 2);

            int vehicleId;
            ss>> vehicleId >> job_size >> max_size; 

            if(naiTable.find(wsm->getRecipientAddress()) && naiTable.NAI_map[wsm->getRecipientAddress()].hopNum == 1)       // relay
            {
                wsm->setSenderAddress(myId);
                sendDown(wsm->dup());
                break;
            }

            if(wsm->getRecipientAddress() == myId)                                          // if this EREP is for the node
            {
                for(int i = job_size; i < job_size + max_size; i ++)                            // obtain correct position
                {
                    double bid;
                    ss>> bid;
                    job_vector[i].bid.insert(pair<int, double>(vehicleId, bid));       // insert bid information for later scheduling
                }        
            }

        }
        break;
    case PROCESSOR:
        if(phase == 0)
        {
            formal_out("EREQ...", 2);

            int vehicleId;
            double vx, vy, vz;
            ss>> vehicleId >> vx >> vy >> vz;

            Coord sourceSpeed(vx,vy,vz);
            if(curSpeed.distance(sourceSpeed) > speedLimit)                                      // compare the speed
            { 
                formal_out("Speed difference too large!", 3);
                // delete(wsm);
                break;
            }

            if(naiTable.find(vehicleId) && naiTable.NAI_map[vehicleId].hopNum == 1)              // 1 hop node forward EREQ
            {
                wsm->setSenderAddress(myId);
                sendDown(wsm->dup());
                break;
            }
            // bool compatibility = rand() < 0.75*RAND_MAX? true:false;                             // consider compatibility
            bool compatibility = true;

            if(compatibility)
                send_EREP(myId, vehicleId, ss);   

        }
        else

            break;
    default: ;
    }               
    v0.push_back(job_size);
    v0.push_back(max_size);
    return v0;
    
}

vector<int> MyVeinsApp::sch(vector<int> v0)
{
    int job_size = v0.at(0), max_size = v0.at(1);
    
    // enter scheduling phase and then begin data transmission
    formal_out("Entering scheduling phase!", 3);
    vector<job> temp;
    temp.assign(job_vector.begin() + job_size, job_vector.begin() + job_size + max_size);
    return scheduling(temp, 0);

}

void MyVeinsApp::dat(int phase, stringstream &ss, WaveShortMessage* wsm, vector<int> serviceCar = vector<int>())
{
    switch(node_type)
    {
    case REQUESTER:
        if(phase == 0)
        {
            idleState = false;
            for(int i = 0; i < serviceCar.size(); i ++)
            {
                job_vector[i].start = simTime().dbl();                            // start of delay
                send_data(job_vector[i], serviceCar[i], 3, simTime());           
            }
            idleState = true;   
            TxEnd = true;
        }
        else
        {
            formal_out("data received...", 2);
            int vehicleId;                      // Id of the requester
            ss>> vehicleId;
            job myJob = work_info[vehicleId];   // get info of current job 

            ss.seekg(-2, ss.end);
            string temp; 
            ss>> temp;
            
            if(temp == "ed")
           {
                EV<<"--- Finish receiving result data from sender "<< vehicleId <<" !\n";    // maybe add time calculation later
                myJob.delay = simTime() - myJob.start;
                job_delay = myJob.delay;

                // emit(sig, job_delay);                                                              // use signal to record the delay of each job
                // recordScalar("job_delay", job_delay);
                delayVec.record(job_delay);  
            }

        }
        break;
    case PROCESSOR:
        if(phase == 0)
        {
            // should decode wdata to get dataSize, resultSize and workload here and send back result of the size
            if(wsm->getRecipientAddress() != myId) 
            {
                wsm->setSenderAddress(myId);
                sendDown(wsm->dup());
                break;            
            }

            // if processor: then use work_info to process; if requester: use it to calculate delay
            formal_out("job brief...", 2);
            job myJob;
            int vehicleId;
            ss>> vehicleId >> myJob.data_size >> myJob.result_size >> myJob.workload >> myJob.utility >> myJob.bid[myId] >> myJob.start;        // index of bid doesn't matter
            work_info.insert(pair<int, job>(vehicleId, myJob));                                                // store source Id and the job in map!!!

        }
        else
        {
            formal_out("data received...", 2);
            int vehicleId;                      // Id of the requester
            ss>> vehicleId;
            job myJob = work_info[vehicleId];   // get info of current job 
            
            ss.seekg(-2, ss.end);
            string temp; 
            ss>> temp;
            if(temp == "ed")   
            {
                // add the size information to processing queueing which is running all the time
                if(current_task_time > simTime())
                    current_task_time += (double) myJob.workload / computing_speed;           // here the computing speed can also be specified for different cars
                else
                {
                    idleState = false;
                    current_task_time = simTime() + (double)myJob.workload / computing_speed;
                }

                // send back result data
                send_data(myJob.result_size, vehicleId, 3, current_task_time);
                work_info.erase(vehicleId);

            }
        }
    default: ;

    }
}


void MyVeinsApp::send_EREQ(queue<job> job_queue, double job_time)
{
    formal_out("sending EREQ...", 3);

    stringstream EREQ;
    EREQ<<"Q "<< myId <<' '<< curSpeed.x <<' '<< curSpeed.y <<' '<< curSpeed.z <<' ';

    // set the index of job segment
    int max_size = job_queue.size();
    int job_size = job_vector.size();
    EREQ<< job_size <<' '<< max_size<<' ';          // identify the position when coming back

    // write data to EREQ
    for(int i = 0; i < max_size; i ++)
    {
        // calculate utility function
        job temp = job_queue.front();
        temp.utility = temp.workload / (temp.workload + job_time);
        job_time -= temp.workload;

        // add temp to a vector before data sending
        job_vector.push_back(temp);

        // writ the brief into a string: only workload in EREQ
        EREQ<< temp.workload <<' ';
        job_queue.pop();
    }


    // send the message: 4k = 166 job, suppose wsm length is enough
    WaveShortMessage * myEREQ = new WaveShortMessage();
    populateWSM(myEREQ);
    myEREQ->setWsmData(EREQ.str().c_str());
    scheduleAt(simTime(), myEREQ);

}

void MyVeinsApp::send_EREP(int vehicleId, int rcvId, stringstream &EREQ)
{
    formal_out("sending EREP...", 3);

    stringstream EREP;
    int nump = 0, job_size, max_size;
    EREQ >> job_size >> max_size;
    EREP <<"P "<<' '<< vehicleId <<' '<< job_size <<' '<< max_size <<' ';       // add job size and max size to position the job vector

    // calculate bids for each job
    while(!EREQ.eof())
    {
        double workload;
        EREQ >> workload;
        double bid = workload / computing_speed;
        EREP << bid <<' ';
        nump ++;
    }
    int k = nump;

    // send to rcvId
    WaveShortMessage * myEREP = new WaveShortMessage();
    populateWSM(myEREP);
    myEREP->setRecipientAddress(rcvId);
    myEREP->setWsmData(EREP.str().c_str());
    scheduleAt(simTime(), myEREP);


}

void MyVeinsApp::generate_job(double lambda, int data_size, int result_size, double workload)
{
    // formal_out("generating jobs...", 3);

    // push job to queue
    job myJob = {data_size, result_size, workload};
    job_queue.push(myJob);
    string str;
    str = "queue length: " + to_string(job_queue.size()) + " ; last job: " + to_string(job_queue.back().workload);
    formal_out(str.c_str(), 3);

    // send wsm to generate next job
    WaveShortMessage * jobMsg = new WaveShortMessage();
    populateWSM(jobMsg);
    jobMsg->setKind(GENERATE_JOB_EVT);
    //double x = jobInterval_dstrb(job_g);
    double x = 1/lambda; 

    scheduleAt(simTime() + x, jobMsg);


}

void MyVeinsApp::send_beacon(vector<int> hop1_Neighbor)         // send once, print all the information into one string of wsm data
{
    // force converting each parameter into stringstream for test now

    stringstream ss;
    ss<<"B "<< myId <<' '<< curSpeed.x <<' '<< curSpeed.y <<' '<< curSpeed.z <<' '<< idleState <<' ';
    for(int i = 0; i < hop1_Neighbor.size(); i ++)
        ss << hop1_Neighbor.at(i) <<' ';


    // send beacon now
    WaveShortMessage * bc = new WaveShortMessage();
    populateWSM(bc);
    bc->setWsmData(ss.str().c_str());
    scheduleAt(simTime(), bc->dup());

    // schecule next beacon: seems easy to schedule it here
    bc->setKind(SEND_MY_BC_EVT);
    scheduleAt(simTime() + my_bc_interval, bc);

}

vector<int> MyVeinsApp::scheduling(vector<job> job_vector, int type)             // schedule each job according to their job briefs
{
    formal_out("scheduling...", 3);

    vector<int> serviceCar;
    for(int i = 0; i < job_vector.size(); i ++)
    {
        map<int, double> bid = job_vector[i].bid;
        vector<pair<int, double>> bid_vec(bid.begin(), bid.end());
        int ri;
        switch(type)
        {
        case 0:             // choose randomly
            //ri = rand() % static_cast<int>(bid.size());             // cause error code 136 if bid.size() = 0
            ri = 10000 % static_cast<int>(bid.size());                // for debug, eliminate the probability

            serviceCar.push_back(bid_vec[ri].first);      
            break;
        case 1:             // choose the vehicle who has the least bid
            sort(bid_vec.begin(), bid_vec.end(), cmp);
            serviceCar.push_back(bid_vec[0].first);
        default: ;
        }
    }  
    return serviceCar;  
}

void MyVeinsApp::send_data(int size, int rcvId, int serial, simtime_t time)        // send data of size wave short messages, containing schedule continuously
{
    //formal_out("sending data...", 3);

    int max_size = 4095;                    // max size of wsm data, except "D" at the start
    int num = (size + 2)/max_size + 1;      // number of wsm, 2 is "ed"
    int last_size = (size + 2)% max_size;   // last wsm size

    for(int i = 0; i < num; i ++)
    {
        int l = max_size;
        if(i == num - 1) l = last_size;

        stringstream ss;
        ss<<"D "<< myId <<' ';
        for(int j = ss.str().size(); j < l; j ++) ss<< '0';
        if(i == num - 1) ss<< "ed";

        WaveShortMessage * data = new WaveShortMessage();
        populateWSM(data, rcvId, serial);
        data->setKind(SEND_DATA_EVT);           // cooperate with handleSelfMsg
        data->setWsmLength(l);
        data->setWsmData(ss.str().c_str());
        scheduleAt(time + i*slot, data);        // send each wsm in every slot, also could be other method
        if(current_task_time <= simTime())      // if processor becomes idle after sending data
            idleState = true;
    }

}

void MyVeinsApp::send_data(job myJob, int rcvId, int serial, simtime_t time)        // overload for requester sending data with job brief
{
    formal_out("sending data with job brief...", 3);

    // send first wsm contaning brief info of the job
    stringstream ss;
    ss<<"J "<< myId <<' '<< myJob.data_size <<' '<< myJob.result_size <<' '<< myJob.workload <<' '<< myJob.utility <<' '<< myJob.bid[rcvId] <<' '<<myJob.start;
    WaveShortMessage * pre_data = new WaveShortMessage();
    populateWSM(pre_data, rcvId, serial);
    pre_data->setKind(SEND_DATA_EVT);
    pre_data->setWsmData(ss.str().c_str());
    scheduleAt(time, pre_data);

    // send data of job's data size
    send_data(myJob.data_size, rcvId, serial, time + slot);

}

void MyVeinsApp::initialize(int stage) {
    formal_out("Initializing...", 1);

    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << endl;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        node_type = rand() < 0.5*RAND_MAX? REQUESTER:PROCESSOR;              // can be initialized by the position or other properties of nodes

        // node_type = myId % 3 == 0? REQUESTER:PROCESSOR;

        current_task_time = 0;
        computing_speed = 1e3;                          // can be specified by our demand
        idleState = true;
        TxEnd = true;
        naiTable.push_back(myId, idleState, 0, simTime() +  200 * my_bc_interval);       // my ID should never expired?
        // sig = registerSignal("sig");

    }
    else if (stage == 1) {
        //Initializing members that require initialize other modules goes here
        vector<int> initialId(1,myId);
        if(node_type == PROCESSOR)
            send_beacon(initialId);          // cannot get myid? need some other operation?
        else
        {
            generate_job(lambda, 1, 1, 0);   // virtual job, to starting normal job caching
            t_disc = simTime().dbl();
        }
    }
}

void MyVeinsApp::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void MyVeinsApp::onBSM(BasicSafetyMessage* bsm) {
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here

}

void MyVeinsApp::onWSM(WaveShortMessage* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples, (

    bool cond = simTime() > 5.0005;        // 4.3, 3.300407, 4.828
    if(cond){

        EV << "current time is " << simTime().dbl() << endl;   // effective point
    }

    if(wsm->getWsmLength() < 50)
        formal_out(wsm->getWsmData(), 1);
    //EV<<wsm->getWsmData()<<"\n\n";

    // for test: both work
    job_delay = job_delay == 0.01? 0.02:0.01;
    delayVec.record(job_delay);             
    // emit(sig, job_delay); 

    // try to display the last delay on the canvas
    stringstream ss_delay;
    ss_delay<<"last delay: "<< job_delay.dbl(); 

    char* wdata = new char[strlen(wsm->getWsmData()) - 1];        // data without my header "T", "B" and "D"
    EV << "strlen of wsm data: " << strlen(wsm->getWsmData()) << endl;
    strcpy(wdata, wsm->getWsmData() + 1);                               // need testing !!!
    stringstream ss;
    ss << string(wdata);    

    /*
    char header;
    stringstream ss;
    ss << string(wsm->getWsmData());
    ss >> header;
     */
 
    // debug only
    if(wsm->getWsmData()[0] != 'B') return;
    
    
    
    switch(wsm->getWsmData()[0]) {
    case 'T':  {  // original code for traffic information, unchanged
        formal_out("traffic info...", 2);

        findHost()->getDisplayString().updateWith("r=16,green");

        if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wdata, 9999);
        if (!sentMessage) {
            sentMessage = true;
            //repeat the received traffic update once in 2 seconds plus some random delay
            wsm->setSenderAddress(myId);
            wsm->setSerial(3);
            // scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
            scheduleAt(simTime() + 2.1, wsm->dup());
        }
        break;
    }
    case 'B':  { // AVE beacon: not relayed, can be received by processor              
        bea(wsm, &ss);
        break;
    }
    case 'Q':{
        if(node_type == REQUESTER) break;
        dis(0, wsm, &ss);                                 // send back to the requester
        break;
    }
    case 'P':{
        if(node_type == PROCESSOR) break;
        vector<int> v0 = dis(1, wsm, &ss);
        if(v0.at(0) < 0 || v0.at(1) < 0) 
        {
            formal_out("Error: no job size, max size stored in vector v0!", 1);
            exit(0);
        }
        vector<int> serviceCar;
        if( simTime().dbl() - t_disc >= rtd)        // if time expired, enter scheduling state
            serviceCar = sch(v0);    
        else break;

       // dat(0, ss, wsm, serviceCar);                     // enter data transmission phase based on service car
        break;
    }
    case 'J':{
      //  dat(0, ss, wsm);
        break;
    }      
    case 'D': {  // data
        // should decode wdata to get dataSize, resultSize and workload here and send back result of the size
        if(wsm->getRecipientAddress() != myId) 
        {
            wsm->setSenderAddress(myId);
            sendDown(wsm->dup());
            break;                
        }

      //  dat(1, ss, wsm);
    }

    default: ;
    }
}

void MyVeinsApp::onWSA(WaveServiceAdvertisment* wsa) {
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

void MyVeinsApp::handleSelfMsg(cMessage* msg) {
    //BaseWaveApplLayer::handleSelfMsg(msg);      // should not be called twice!!
    //this method is for self messages (mostly timers)
    //it is important to call the BaseWaveApplLayer function for BSM and WSM transmission  // BSM and WSA?

    formal_out("Handling self message...", 1);

    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {         // if the pointer exists or not? I think most wsm case is this one?
        
        // debug only
        if(msg->getKind() == GENERATE_JOB_EVT && job_queue.size() > naiTable.value + 1 && TxEnd)
        {
            t_disc = simTime().dbl();               // update the begin time of discovery
            formal_out("Job caching end!", 2);      // avoid entering discovery stage, then shall no data transmission
            return;
        }

        
        
        switch(msg->getKind()) {
        case SEND_DATA_EVT:
        {
            formal_out("send data...", 2);

            //not send the wsm for several times like traffic update
            sendDown(wsm->dup());                 // should consider resend or not?
            delete(wsm);
            if(current_task_time < simTime())     // maybe here when the requestor finishing sending data  
                idleState = true;
            break;
        }
        case SEND_MY_BC_EVT:
        {
            bea(wsm);          
            break;
        }
        case GENERATE_JOB_EVT:
        {

            // if job caching ends: enter discovery directly
            if(job_queue.size() > naiTable.value + 1 && TxEnd)              // job caching end condition
            {
                dis(0, wsm);
                t_disc = simTime().dbl();   // update the begin time of discovery
            }
            cac();

            break;
        }
        default: sendDown(wsm->dup());
        /*
        {
            //send this message on the service channel until the counter is 3 or higher.
            //this code only runs when channel switching is enabled                  // what's it?
            // formal_out("default...", 2);

            sendDown(wsm->dup());
            wsm->setSerial(wsm->getSerial() +1);
            if (wsm->getSerial() >= 3) {
                //stop service advertisements
                stopService();
                delete(wsm);
            }
            else {
                scheduleAt(simTime()+1, wsm);
            }
        }*/
        }
        
    }
    else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }

}

void MyVeinsApp::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    // for test
    //    if (mobility->getSpeed() < 1) {                                 // stopped for for at least 10s? not used in my loop version because both cars run all the time
    //        if (simTime() - lastDroveAt >= 0 && sentMessage == false) {
    //
    //
    formal_out("Handling position update...", 1);

    findHost()->getDisplayString().updateWith("r=16,red");
    sentMessage = true;

    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm);
    string tr_wsm = "T" + mobility->getRoadId();

    wsm->setWsmData(tr_wsm.c_str());

    //host is standing still due to crash
    if (dataOnSch) {
        startService(Channels::SCH2, 42, "Traffic Information Service");
        //started service and server advertising, schedule message to self to send later
        scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
    }
    else {
        //send right away on CCH, because channel switching is disabled
        sendDown(wsm);
    }
    //        }
    //    }
    //    else {
    //        lastDroveAt = simTime();
    //    }
}
