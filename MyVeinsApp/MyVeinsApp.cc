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
#include <vector>
#include <random>
#include <boost/thread.hpp>
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
*/

// job parameters
double job_time = 0;
double lambda = 50;
double dataSize = 100e3;
double resultSize = 100e3;
double work_mean = 4;

bool TxEnd;

double speedLimit = 15;         // 15 m/s
simtime_t my_bc_interval = 5;   // 5 s
boost::mutex mtx;               // mtx for reading NAI table: single or multiple thread?


void MyVeinsApp::send_EREQ(std::queue job_queue, double job_time)
{
    string EREQ = "Q" + (string)myId + (string)curSpeed.x + (string)curSpeed.y + (string)curSpeed.z;

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
        EREQ += (string)temp.workload;
        job_queue.pop();
    }

    // send the message: 4k = 166 job, suppose wsm length is enough
    WaveShortMessage * myEREQ = WaveShortMessage();
    populate(myEREQ);
    myEREQ->setWsmData(EREQ.c_str());
    scheduleAt(simTime(), myEREQ);

}

void MyVeinsApp::send_EREP(int vehicleId, int rcvId, string str)
{
    // calculate bids for each job
    string EREP = "P" + (string)vehicleId;
    for(int i = 0; i < (int)job_brief.size()/ (2*(sizeof(int) + sizeof(double))); i ++)
    {
        double workload = str.substr(0, sizeof(double)).c_str(); 
        str = str.substr(sizeof(double));
        double bid = workload / computing_speed;                        // haven't considered just sharing part of computing resource
        EREP += (string)bid;
    }
    
    // send to rcvId
    WaveShortMessage * myEREP = WaveShortMessage();
    populate(myEREP);
    myEREP->setRecipientAddress(rcvId);
    myEREP->setWsmData(EREP.c_str());
    scheduleAt(simTime(), myEREP);
    
    
}

void MyVeinsApp::generate_job(double lambda, int data_size, int result_size, double workload)
{
    // push job to queue
    job myJob = {data_size, result_size, workload};
    job_queue.push(myJob);

    // generate interval obey exponential distribution: seems not work now?
    std::default_random_engine generator;
    std::exponential_distribution<double> distribution(lambda);

    // send wsm to generate next job
    WaveShortMessage * jobMsg = WaveShortMessage();
    populate(jobMsg);
    jobMsg->setKind(GENERATE_JOB_EVT);
    scheduleAt(simTime() + distribution(generator), jobMsg);


}

void MyVeinsApp::send_beacon(std::vector<int> hop1_Neighbor)         // send once, print all the information into one string of wsm data
{
    // force converting each parameter into string for test now

    string str = 'B' + (string)myId + (string)curSpeed.x + (string)curSpeed.y + (string)curSpeed.z + (string)idleState;      // string may have some offload?
    for(int i = 0; i < hop1_Neighbor.size(); i ++)
        str += (string)hop1_Neighbor.at(i);

    // send beacon now
    WaveShortMessage * bc = WaveShortMessage();
    populateWSM(bc);
    bc->setWsmData(str.c_str());
    scheduleAt(simTime(), bc->dup());

    // schecule next beacon: seems easy to schedule it here
    bc->setKind(SEND_MY_BC_EVT);
    scheduleAt(simTime() + my_bc_interval, bc);

}

void MyVeinsApp::send_data(int size, int rcvId, int serial, simTime time)        // send data of size wave short messages, containing schedule continuously
{
    int max_size = 4095;                    // max size of wsm data, except "D" at the start
    int num = (size + 2)/max_size + 1;      // number of wsm, 2 is "ed"
    int last_size = (size + 2)% max_size;   // last wsm size
    double slot = 0.05;

    for(int i = 0; i < num; i ++)
    {
        int l = max_size;
        if(i == num - 1) l = last_size;

        string str;
        for(int j = 0; j < l; j ++) str+= '0';
        str = "D" + str;
        if(i == num - 1) str += "ed";

        WaveShortMessage * data = WaveShortMessage();
        populateWSM(data, rcvId, serial);
        data->setKind(SEND_DATA_EVT);           // cooperate with handleSelfMsg
        data->setWsmLength(l);
        data->setWsmData(str.c_str());
        scheduleAt(time + slot, data);          // send each wsm in every slot, also could be other method
        if(current_task_time <= simTime())      // if processor becomes idle after sending data
            idleState = true;

    }

}

void MyVeinsApp::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        node_type = rand() < 0.5*RAND_MAX;              // can be initialized by the position or other properties of nodes
        current_task_time = 0;
        computing_speed = 1e3;
        idleState = true;
        naiTable.push_back(myId, idleState, 0, simTime() + my_bc_interval);

    }
    else if (stage == 1) {
        //Initializing members that require initialize  break;d other modules goes here
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

    // example code only handles the position changing message... so I must modify the whole code here because of header for identification
    EV<<"Receiving WSM:"<<std::endl;
    EV<<wsm->getWsmData()<<"\n\n";

    const char* wdata = new char[strlen(wsm->getWsmData()) - 3];        // data without my header "T", "B" and "D"
    wdata = wsm->getWsmData() + 1;                                      // aukward code here..

    switch(wsm->getWsmData()[0]) {
    case 'T':    // original code for traffic information, unchanged
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

    case 'B':   // AVE beacon
        if(node_type == PROCESSOR) break;           // only requesters care about AVE beacon
        string str(wdata);

        // decode the information of beacon
        int vehicleId = str.substr(0, sizeof(int)).c_str(); str = str.substr(sizeof(int));
        double vx = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        double vy = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        double vz = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        bool ifIdle = str.substr(0, sizeof(bool)).c_str(); str = str.substr(sizeof(bool));
        simtime_t expiredTime = simTime() + my_bc_interval;

        // compare the speed
        Coord sourceSpeed(vx,vy,vz);
        if(curSpeed.distance(sourceSpeed) > speedLimit)
        {
            delete(wsm);
            break;
        }

        // decode the neighbor ID and update the NAI table
        int hop1_num = (wsm->getWsmLength() - 30) / sizeof(int);            // 30 is length before neighbor part, should be varified
        vector<int> hop1_neighbor;

        for(int i = 0; i < hop1_num; i ++)
        {
            int neighborId = str.substr(0, sizeof(int)).c_str();
            str = str.substr(sizeof(int));

            // mtx.lock();
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
            // mtx.unlock();

        }
        break;

    case 'Q':
        if(node_type == REQUESTER) break;
        string str(wdata);

        // decode the information of EREQ
        int vehicleId = str.substr(0, sizeof(int)).c_str(); str = str.substr(sizeof(int));
        double vx = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        double vy = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        double vz = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));

        Coord sourceSpeed(vx,vy,vz);
        if(curSpeed.distance(sourceSpeed) > speedLimit)                                      // compare the speed
        { 
            delete(wsm);
            break;
        }

        if(naiTable.find(vehicleId) && naiTable.NAI_map[vehicleId].hopNum == 1)              // 1 hop node forward EREQ
            sendDown(wsm->dup());
        
        bool compatibility = rand() < 0.75*RAND_MAX? true:false;                             // consider compatibility
        if(compatibility)
            send_EREP(myId, vehicleId, str);                                                 // send back to the requester
        break;
        
    case 'P':
        if(node_type == PROCESSOR) break;
        string wdata;
        
        // decode the information of EREP
        int vehicleId = str.substr(0, sizeof(int)).c_str();                                 // note: here vehicleId may not be the sender but the source of EREP
        str = str.substr(sizeof(int));
        
        if(naiTable.find(wsm->getRecipientAddress()) && naiTable.NAI_map[wsm->getRecipientAddress()].hopNum == 1)       // relay
            sendDown(wsm->dup());
        else if(wsm->getRecipientAddress() = myId)                                          // if this EREP is for the node
        {
            for(int i = 0; i < str.size() / sizeof(double); i ++)
            {
                double bid = str.substr(0, sizeof(double)).c_str();
                str = str.substr(sizeof(double));
                job_vector[i].bid.insert(std::pair<int, double>(vehicleId, bid));           // insert bid information for later scheduling
            }        
        }
        break;
        
    case 'D':   // data
        // should decode wdata to get dataSize, resultSize and workload here


        // accumulate the wsm data size to calculate a whole job size
        if(node_type == PROCESSOR)
        {
            int data_length = strlen(wdata) - 2;
            if(data_size.find(wsm->getSenderAddress()) == std::data_size.end())               // if it's the first wsm from this sender (data may consist of many wsm)
                data_size.insert(std::pair<int,int>(wsm->getSenderAddress, data_length ));
            else                                                                              // if not
                data_size.at(wsm->getSenderAddress()) += data_length;

            if(!strcmp((const char*)wdata + strlen(wdata) - 2, "ed"))                         // if this wsm is the end of a complete data
            {

                // add the size information to processing queueing which is running all the time
                int this_size = data_size.at(wsm->getSenderAddress());
                if(current_task_time > simTime())
                    current_task_time += (double)this_size/computing_speed;
                else
                {
                    idleState = false;
                    current_task_time = simTime() + (double)this_size/computing_speed;
                }

                send_data(this_size, wsm->getSenderAddress(), 3, current_task_time);                                   // serial = 3 for now
                data_size.erase(wsm->getSenderAddress());                                                              // delete the sender's record at once to receive next data from the same sender!

            }
        }
        else if (node_type = REQUESTER)
        {
            int data_length = strlen(wdata) - 2;
            if(data_size.find(wsm->getSenderAddress()) == std::data_size.end())                      // results may come from many processor, here data_size mains result data size
                data_size.insert(std::pair<int,int>(wsm->getSenderAddress, data_length ));
            else
                data_size.at(wsm->getSenderAddress()) += data_length;
            if(!strcmp((const char*)wdata + strlen(wdata) - 2, "ed"))
            {
                EV<<"Finish receiving result data from sender "<<wsm->getSenderAddress()<<" !\n";    // maybe add time calculation later
                EV<<"data size: "<<data_length<<" B\n\n";
                data_size.erase(wsm->getSenderAddress());

            }
        }
    }
}

void MyVeinsApp::onWSA(WaveServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

    // example in TraCIDemo means th at change ch
    break;annel according to wsa and not use more information
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
            //not send the wsm for several times like traffic update
            sendDown(wsm->dup());                 // should consider resend or not?
            delete(wsm);
            break;

        case SEND_MY_BC_EVT:
            naiTable.update();
            send_beacon(naiTable.generate_hop1());
            break;

        case GENERATE_JOB_EVT:
            // if job caching ends: enter discovery directly
            if(job_queue.size() > naiTable.value + 1 && TxEnd)              // job caching end condition
            {
                naiTable.update();
                send_EREQ(job_queue, job_time);
                job_time = 0;
            }

            // generate a new job on this node
            std::default_random_engine generator;
            std::exponential_distribution<double> distribution(work_mean);
            double workload = distribution(generator);
            job_time += workload;
            generate_job(lambda, dataSize, resultSize, workload);
            break;

        default:
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
