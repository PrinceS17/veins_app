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
#define SEND_DATA_EVT 2
#define SEND_MY_BC_EVT 3
#define GENERATE_JOB_EVT 4

Define_Module(MyVeinsApp);

/* head of different wsm:
T: traffic msg of original code;
B: my beacon;
Q: EREQ at discovery;
P: EREP at discovery;
D: data at TX;
J: job brief before sending data
*/

// job parameters
double job_time = 0;
double lambda = 50;
double dataSize = 100e3;
double resultSize = 100e3;
double work_mean = 4;
double slot = 0.05;

bool TxEnd;

double speedLimit = 15;         // 15 m/s
double my_bc_interval = 5;   // 5 s

// global variable
simtime_t job_delay;


void MyVeinsApp::send_EREQ(std::queue<job> job_queue, double job_time)
{
    EV << "Sending EREQ!\n\n";
    stringstream EREQ;
    EREQ<<"Q "<< myId <<' '<< curSpeed.x <<' '<< curSpeed.y <<' '<< curSpeed.z <<' ';

    // write data to EREQ
    for(int i = 0; i < job_queue.size(); i ++)
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
    EV << "Sending EREP!\n\n";
    // calculate bids for each job
    stringstream EREP;
    EREP <<"P "<<' '<< vehicleId <<' ';
    while(!EREQ.eof())
    {
        double workload;
        EREQ >> workload;
        double bid = workload / computing_speed;
        EREP << bid <<' ';
        
    }
    
    // send to rcvId
    WaveShortMessage * myEREP = new WaveShortMessage();
    populateWSM(myEREP);
    myEREP->setRecipientAddress(rcvId);
    myEREP->setWsmData(EREP.str().c_str());
    scheduleAt(simTime(), myEREP);
    
    
}

void MyVeinsApp::generate_job(double lambda, int data_size, int result_size, double workload)
{
    EV << "Generating jobs!\n\n";
    
    // push job to queue
    job myJob = {data_size, result_size, workload};
    job_queue.push(myJob);

    // generate interval obey exponential distribution: seems not work now?
    std::default_random_engine generator;
    std::exponential_distribution<double> distribution(lambda);

    // send wsm to generate next job
    WaveShortMessage * jobMsg = new WaveShortMessage();
    populateWSM(jobMsg);
    jobMsg->setKind(GENERATE_JOB_EVT);
    scheduleAt(simTime() + distribution(generator), jobMsg);


}

void MyVeinsApp::send_beacon(std::vector<int> hop1_Neighbor)         // send once, print all the information into one string of wsm data
{
    // force converting each parameter into stringstream for test now
    EV << "Sending my beacon!\n\n";
    
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
    EV << "Sending data!\n\n";
    vector<int> serviceCar;
    for(int i = 0; i < job_vector.size(); i ++)
    {
        map<int, double> bid = job_vector[i].bid;
        vector<pair<int, double>> bid_vec(bid.begin(), bid.end());
        int ri;
        switch(type)
        {
        case 0:             // choose randomly
            ri = rand() % static_cast<int>(bid.size());
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
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        node_type = rand() < 0.5*RAND_MAX? REQUESTER:PROCESSOR;              // can be initialized by the position or other properties of nodes
        
        current_task_time = 0;
        computing_speed = 1e3;                          // can be specified by our demand
        idleState = true;
        TxEnd = false;
        naiTable.push_back(myId, idleState, 0, simTime() + my_bc_interval);
        sig = registerSignal("sig");

    }
    else if (stage == 1) {
        //Initializing members that require initialize other modules goes here
        vector<int> initialId(1,myId);
        if(node_type == PROCESSOR)
            send_beacon(initialId);          // cannot get myid? need some other operation?
    }
}

void MyVeinsApp::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void MyVeinsApp::onBSM(BasicSafetyMessage* bsm) {
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here
    EV<<"Receiving BSM!"<<std::endl;

}

void MyVeinsApp::onWSM(WaveShortMessage* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples, (

    // example code only handles the position changing message... so I must modify the whole code here with headers for identification
    EV<<"Receiving WSM:"<<std::endl;
    EV<<wsm->getWsmData()<<"\n\n";
    
    // for test: both work
    job_delay = job_delay == 0.01? 0.02:0.01;
    delayVec.record(job_delay);             
    emit(sig, job_delay); 
    
    // try to display the last delay on the canvas
    stringstream ss_delay;
    ss_delay<<"last delay: "<< job_delay.dbl();
    bubble(ss_delay.str().c_str());
    

    char* wdata = new char[strlen(wsm->getWsmData()) - 1];        // data without my header "T", "B" and "D"
    strcpy(wdata, wsm->getWsmData() + 1);                               // need testing !!!
    stringstream ss;
    ss<<string(wdata);

    switch(wsm->getWsmData()[0]) {
    case 'T':  {  // original code for traffic information, unchanged
        findHost()->getDisplayString().updateWith("r=16,green");

        if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wdata, 9999);
        if (!sentMessage) {
            sentMessage = true;
            //repeat the received traffic update once in 2 seconds plus some random delay
            wsm->setSenderAddress(myId);
            wsm->setSerial(3);
            scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
        }
        break;
    }
    case 'B':  { // AVE beacon: not relayed
        if(node_type == PROCESSOR) break;           // only requesters care about AVE beacon
        int vehicleId;
        double vx, vy, vz;
        bool ifIdle;
        ss>> vehicleId >> vx >> vy >> vz >> ifIdle;
        simtime_t expiredTime = simTime() + my_bc_interval;
         
        // compare the speed
        Coord sourceSpeed(vx,vy,vz);
        if(curSpeed.distance(sourceSpeed) > speedLimit)
        {
            delete(wsm);
            break;
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
                naiTable.push_back(vehicleId, ifIdle, hopNum, expiredTime);
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
        break;
    }
    case 'Q':{
        if(node_type == REQUESTER) break;
        int vehicleId;
        double vx, vy, vz;
        ss>> vehicleId >> vx >> vy >> vz;
        
        Coord sourceSpeed(vx,vy,vz);
        if(curSpeed.distance(sourceSpeed) > speedLimit)                                      // compare the speed
        { 
            delete(wsm);
            break;
        }

        if(naiTable.find(vehicleId) && naiTable.NAI_map[vehicleId].hopNum == 1)              // 1 hop node forward EREQ
        {
            wsm->setSenderAddress(myId);
            sendDown(wsm->dup());
        }
        bool compatibility = rand() < 0.75*RAND_MAX? true:false;                             // consider compatibility
        if(compatibility)
            send_EREP(myId, vehicleId, ss);                                                 // send back to the requester
        break;
    }
    case 'P':{
        if(node_type == PROCESSOR) break;
        int vehicleId;
        ss>> vehicleId; 
        
        if(naiTable.find(wsm->getRecipientAddress()) && naiTable.NAI_map[wsm->getRecipientAddress()].hopNum == 1)       // relay
        {
            wsm->setSenderAddress(myId);
            sendDown(wsm->dup());
        }
        else if(wsm->getRecipientAddress() == myId)                                          // if this EREP is for the node
        {
            int i = 0;
            while(!ss.eof())
            {
                double bid;
                ss>> bid;
                job_vector[i].bid.insert(std::pair<int, double>(vehicleId, bid));           // insert bid information for later scheduling
                i ++;
            }        
        }
        
        // enter scheduling phase and then begin data transmission
        vector<int> serviceCar = scheduling(job_vector, 0);
        idleState = false;
        for(int i = 0; i < serviceCar.size(); i ++)
        {
            job_vector[i].start = simTime().dbl();                            // start of delay
            send_data(job_vector[i], serviceCar[i], 3, simTime());
        }
        idleState = true;   
        TxEnd = true;
        
        break;
    }
    case 'J':{
        // should decode wdata to get dataSize, resultSize and workload here and send back result of the size
        if(wsm->getRecipientAddress() != myId) 
        {
            wsm->setSenderAddress(myId);
            sendDown(wsm->dup());
            break;            
        }
        
        // if processor: then use work_info to process; if requester: use it to calculate delay
        job myJob;
        int vehicleId;
        ss>> vehicleId >> myJob.data_size >> myJob.result_size >> myJob.workload >> myJob.utility >> myJob.bid[myId] >> myJob.start;        // index of bid doesn't matter
        work_info.insert(std::pair<int, job>(vehicleId, myJob));                                                // store source Id and the job in map!!!
        
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
       
        int vehicleId;                      // Id of the requester
        ss>> vehicleId;
        job myJob = work_info[vehicleId];   // get info of current job
        
        // accumulate the wsm data size to calculate a whole job size
        if(node_type == PROCESSOR)
        {
               
//            int data_length = strlen(wdata) - 2;
//            if(data_size.find(wsm->getSenderAddress()) == std::data_size.end())               // if it's the first wsm from this sender (data may consist of many wsm)
//                data_size.insert(std::pair<int,int>(wsm->getSenderAddress, data_length ));
//            else                                                                              // if not
//                data_size.at(wsm->getSenderAddress()) += data_length;
            bubble("Processor!");
            if(!strcmp((const char*)wdata + strlen(wdata) - 2, "ed"))                         // if this wsm is the end of a complete data
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
        else if (node_type == REQUESTER)
        {
            // need to prevent it entering another phase?            
            bubble("Requester!");
           
            if(!strcmp((const char*)wdata + strlen(wdata) - 2, "ed"))                                // hope this line works well!
            {
                EV<<"Finish receiving result data from sender "<<wsm->getSenderAddress()<<" !\n";    // maybe add time calculation later
                myJob.delay = simTime() - myJob.start;
                job_delay = myJob.delay;
                
                emit(sig, job_delay);                                                              // use signal to record the delay of each job
                recordScalar("job_delay", job_delay);             
            }
        }
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

    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {         // if the pointer exists or not? I think most wsm case is this one?
        switch(msg->getKind()) {
        case SEND_DATA_EVT:
        {
            //not send the wsm for several times like traffic update
            sendDown(wsm->dup());                 // should consider resend or not?
            delete(wsm);
            if(current_task_time < simTime())     // maybe here when the requestor finishing sending data  
                idleState = true;
            break;
        }
        case SEND_MY_BC_EVT:
        {
            naiTable.update();
            send_beacon(naiTable.generate_hop1());
            break;
        }
        case GENERATE_JOB_EVT:
        {
            // if job caching ends: enter discovery directly
            if(job_queue.size() > naiTable.value + 1 && TxEnd)              // job caching end condition
            {
                naiTable.update();
                send_EREQ(job_queue, job_time);                             // queue popped in send_EREQ
                job_time = 0;
                TxEnd = false;
            }

            // generate a new job on this node
            std::default_random_engine generator;
            std::exponential_distribution<double> distribution(work_mean);
            double workload = distribution(generator);
            job_time += workload;
            generate_job(lambda, dataSize, resultSize, workload);
            break;
        }
        default:
        {
           //send this message on the service channel until the counter is 3 or higher.
           //this code only runs when channel switching is enabled                  // what's it?
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
       }
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
            findHost()->getDisplayString().updateWith("r=16,red");
            sentMessage = true;

            WaveShortMessage* wsm = new WaveShortMessage();
            populateWSM(wsm);
            std::string tr_wsm = "T" + mobility->getRoadId();

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
