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
#include <random>
#include "TaskOffload.h"
using namespace std;

#define selfB 2
#define selfG 3
#define selfDup 4
#define selfR 5
#define onB 11
#define onT 12
#define onD 13
#define onJ 14


Define_Module(TaskOffload);

// following are tool functions

void TaskOffload::formal_out(const char* str, int lv)
{
    switch(lv)
    {
    case 1:                     // event like onWSM
        EV<<" "<< str <<"\n";
        break;
    case 2:                     // option like send beacon
        EV<<"    "<< str <<"\n";
        break;
    case 3:                     // specific function like send EREQ
        EV<<"       "<< str <<"\n";
        break;
    default: ;
    }
}

void TaskOffload::display_SeV()
{
    formal_out("SeV information:", 2);
    string str = "scale = 1e" + to_string((int)log10(scale));
    formal_out(str.c_str(), 2);
    formal_out("id     delay/bit (us)      count     occur time     last time", 3);
    for(auto id:SeV_info.SeV_set)
    {
        string str = to_string(id) + "      " + to_string(SeV_info.bit_delay.at(id) * 1e6) + "              " + to_string(SeV_info.count.at(id)) + "       " + to_string(SeV_info.occur_time.at(id)) + "        " + to_string(SeV_info.last_time.at(id));
        formal_out(str.c_str(), 3);
    }    
}

int TaskOffload::nextKind(int kind)
{
    if(node_type == SeV)
    {
        switch(kind)
        {
        case onJ: return onD;
        case onD: return selfR;
        case selfR: return onJ;
        case 0: return onJ;         // start when there's no entry
        default: return -1;         // error kind
        }
    }
    else
    {
        switch(kind)
        {
        case selfG: return onD;
        case onD: return selfG;
        case 0: return onD;         // in fact no need
        default: return -1;
        }
    }
}

void TaskOffload::relay(WaveShortMessage* wsm)
{
    wsm->setSenderAddress(myId);
    wsm->setSerial(wsm->getSerial() +1);
    sendDown(wsm->dup());
    string str = "Relay! Current serial is " + to_string(wsm->getSerial());
//    formal_out(str.c_str(), 3);
}

bool TaskOffload::on_data_check(WaveShortMessage* wsm, int srcId)
{
    if(wsm->getSenderAddress() != myId)                     // it's not a self-msg, avoid printing relay for sending result
    {
        if(srcId == myId) 
        {   
//            formal_out("Discard self message!", 1); 
            return false; 
        }
        if(wsm->getRecipientAddress() != myId)
        {   
            relay(wsm); 
            return false;
        }
    }
    int curKind;
    if(bp_list.find(srcId) != bp_list.end())                
        curKind = bp_list.at(srcId);
    else curKind = 0;                                       // for SeV, the initial stage sets 0
    if(wsm->getKind() != nextKind(curKind))                 // pass only it's the right kind
    {
        string str = "Block by block-pass list! Current kind: " + to_string(curKind) + "; msg kind: " + to_string(wsm->getKind());
//        formal_out(str.c_str(), 1);         // e.g. bp_list: (13, onJ) means: for 13, only pass (13, onD)
        return false;
    }
    return true;
}

bool TaskOffload::checkWSM(WaveShortMessage* wsm)
{
    if(wsm->getSerial() >= 3)                       // discard 3-hop message or self message
    {
//        formal_out("Discard 3-hop message!", 1);
        return false;
    }

    if(NULL == wsm->getWsmData()) {formal_out("Error: void pointer!", 1); return false;}              // check if void pointer
    if(strlen(wsm->getWsmData()) <= 1) {formal_out("Error: receive too little data!", 1); return false;}       // check data length   
    return true;
}

int TaskOffload::scheduling(double beta, double x_t)           // AVUCB algorithm first
{
    formal_out("scheduling...", 2);
    vector<double> utility;
    for(int i = 0; i < SeV_info.SeV_set.size(); i ++)
    {
        int id = SeV_info.SeV_set.at(i);
        double xt1 = x_t > mean_size ? 1.0:0.0;
        double d_tn = simTime().dbl() - SeV_info.occur_time.at(id);
        double k = SeV_info.count.at(id);
        utility.push_back( SeV_info.bit_delay.at(id) * scale - sqrt(beta * (1 - xt1) * log(d_tn) / k) );

        string str = "SeV " + to_string(id) + ": u = " + to_string(utility.at(i));
        formal_out(str.c_str(), 3);
    }
    int vid = SeV_info.SeV_set[min_element(utility.begin(), utility.end()) - utility.begin()];
    string str = "Choice: " + to_string(vid);
    formal_out(str.c_str(), 3);
    return vid;
}

void TaskOffload::local_process(task myTask)
{
    formal_out("locally_process...", 3);

    //    CPU_percentage = uniform(0.2, 0.5);
    double task_time =  myTask.data_size * myTask.cycle_per_bit / (CPU_freq_max * CPU_percentage);
    if(current_task_time > simTime())
        current_task_time += task_time;
    else
    {
        idle_state = false;
        current_task_time = simTime() + task_time;
    }    
    myTask.delay = current_task_time.dbl() - myTask.start;        // recored the final delay
    job_delay = myTask.delay;
    // emit(sig, job_delay);                                         // commented when caring about offloading delay
}

void TaskOffload::send_data(double size, int rcvId, int serial)
{
    string str = "sending data of size " + to_string((int)size / 1000) + " Kbit ...";
    formal_out(str.c_str(), 3);
    int max_size = 4096;                         // max size of wsm data, except "D" at the start
    int num = ((int)size + 2)/max_size + 1;      // number of wsm, 2 is "ed"
    int last_size = ((int)size + 1)% max_size + 1;   // last wsm size
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
        if(i == num - 1) 
            for(int i:{1,2,3}) sendDown(data->dup());
        else sendDown(data->dup());                      // commented it can obviously increase the speed
        delete(data);
        if(current_task_time <= simTime()) idle_state = true;
    }
}

void TaskOffload::send_data(task myTask, int rcvId, int serial)
{
    formal_out("sending task brief...", 3);
    stringstream ss;
    ss<< myId <<' '<< myTask.data_size <<' '<< myTask.result_size <<' '<< myTask.cycle_per_bit <<' '<<myTask.start;
    WaveShortMessage* pre_data = new WaveShortMessage();
    populateWSM(pre_data, rcvId, serial);
    pre_data->setKind(onJ);
    pre_data->setWsmData(ss.str().c_str());
    for(int i :{1,2,3,4,5,6})                         // resend 6 times to ensure the reliability!
        sendDown(pre_data->dup());
    send_data(myTask.data_size, rcvId, serial);
}

// following are handlers

void TaskOffload::handleTraffic(WaveShortMessage* wsm)
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

void TaskOffload::handleBeacon(WaveShortMessage* wsm)
{

    formal_out("TaV: my beacon...", 2);
    string str = "my SeV now: ";
    for(int id:SeV_info.SeV_set)
        str += to_string(id) + " ";
    formal_out(str.c_str(), 2);

    stringstream ss(wsm->getWsmData());
    int vehicleId;
    Coord position, speed;
    bool ifIdle;
    ss >> vehicleId >> ifIdle >> position.x >> position.y >> position.z >> speed.x >> speed.y >> speed.z;
    bool ifAvailable = curPosition.distance(position) < Crange && curSpeed.distance(speed) < speed_limit;
    if(SeV_info.if_exist(vehicleId) && ifAvailable)
        SeV_info.last_time.at(vehicleId) = simTime().dbl();  
    else if(!SeV_info.if_exist(vehicleId) && ifAvailable)
        SeV_info.push_back(vehicleId);
    display_SeV();
}

void TaskOffload::handleOffload(WaveShortMessage* wsm)
{

    formal_out("TaV: generate tasks...", 2);
    //    double x_t = 6e5;
    double x_t = uniform(2, 10) * 1e5;
    task myTask = {x_t, x_t * alpha0, w0, simTime().dbl()};
    task_vector.push_back(myTask);
    WaveShortMessage* tsk = new WaveShortMessage();     // schedule next task 
    populateWSM(tsk);
    tsk->setKind(selfG);
    scheduleAt(simTime() + task_interval, tsk);

    formal_out("TaV: begin offloading...", 2);
    int rcvId = -9999;
    SeV_info.check(work_info);                          // it's supposed to solve the 'work info lost' problem
    
    if(SeV_info.SeV_set.empty())                        // if no SeV currently
    {
        local_process(myTask);
        return;
    }
    for(int id:SeV_info.SeV_set)
    {
        if(!SeV_info.if_connect(id) || SeV_info.SeV_set.size() == 1)   // find one unconnected SeV or only one SeV
        {
            rcvId = id;
            break;
        }
    }   
    if(rcvId == -9999)  rcvId = scheduling(beta, x_t);  // all connected
    send_data(myTask, rcvId, 0);
    work_info[rcvId] = myTask;

    map<int, int>::iterator ite;
    for(ite = bp_list.begin(); ite != bp_list.end(); ite ++)            // allow all data being received
        ite->second = selfG;

}

void TaskOffload::updateResult(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    int vehicleId;
    ss >> vehicleId;
    if(!on_data_check(wsm, vehicleId)) return;
//    formal_out(("result received from " + to_string(vehicleId)).c_str(), 2);
 
    ss.seekg(-2, ss.end);
    string temp;
    ss >> temp;
    
    if(temp == "ed")
    {
        if(work_info.find(vehicleId) == work_info.end())
        {
            formal_out("Already end receiving this result!", 1);
            return;
        }
        task myTask = work_info.at(vehicleId);
        myTask.delay = simTime().dbl() - myTask.start;
        if(scale < 0)
            scale = pow(10, floor(log10(myTask.data_size / myTask.delay)));  // scale the u into [0, 1]
        if(!SeV_info.if_exist(vehicleId)) 
            formal_out(" Error: vehicle doesn't exist in SeV table now!", 1);   // following .count.at will throw an out-of-range error
        if(SeV_info.count.at(vehicleId) == 0)
            SeV_info.init(vehicleId, myTask.delay, myTask.data_size);        // add occurrence time
        else SeV_info.update(vehicleId, myTask.delay, myTask.data_size);     // maintain u & k
        display_SeV();

        job_delay = myTask.delay;
        emit(sig, job_delay);           // send record signal
        string str = "Finish updating result from " + to_string(vehicleId) + "; delay = " + to_string(job_delay.dbl());
        formal_out(str.c_str(), 1);
        work_info.erase(vehicleId);
        bp_list[vehicleId] = onD;
    }
}

void TaskOffload::sendBeacon(WaveShortMessage* wsm)
{
    formal_out("SeV: send beacon...", 2);
    stringstream ss;
    ss<< myId <<' '<< idle_state << curPosition.x <<' '<< curPosition.y <<' '<< curPosition.z <<' '<< curSpeed.x << ' '<< curSpeed.y <<' '<< curSpeed.z <<' ';
    WaveShortMessage* bc = new WaveShortMessage();
    populateWSM(bc);
    bc->setKind(selfDup);
    bc->setWsmData(ss.str().c_str());
    scheduleAt(simTime(), bc->dup());
    bc->setKind(selfB);
    scheduleAt(simTime() + bc_interval, bc->dup());
}

void TaskOffload::processBrief(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    int vehicleId;
    ss >> vehicleId;
    if(!on_data_check(wsm, vehicleId)) return;
    formal_out("process brief...", 2);
    formal_out(ss.str().c_str(), 2);
    task myTask;
    ss >> myTask.data_size >> myTask.result_size >> myTask.cycle_per_bit >> myTask.start;
    work_info[vehicleId] = myTask;
    //    CPU_percentage = uniform(0.2, 0.5);             // randomly change percentage at each period
    bp_list[vehicleId] = onJ;
}

void TaskOffload::processTask(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    int vehicleId;
    ss >> vehicleId;
    if(!on_data_check(wsm, vehicleId)) return;

//    formal_out("task data received...", 2);
    task myTask = work_info.at(vehicleId);
    ss.seekg(-2, ss.end);
    string temp;
    ss >> temp;
    if(temp == "ed")
    {
        if(simTime() > 65.589)
        {
            
            formal_out("65.589!", 1);
        }
        double task_time = myTask.data_size * myTask.cycle_per_bit / (CPU_freq_max * CPU_percentage);
        if(current_task_time > simTime())
            current_task_time += task_time;
        else
        {
            idle_state = false;
            current_task_time = simTime() + task_time;
        } 
        string str = "task time: " + to_string(task_time) + "; schedule time: " + to_string(current_task_time.dbl()) + "; from vehicle: " + to_string(vehicleId);
        formal_out(str.c_str(), 3);
        
        WaveShortMessage* sdr = new WaveShortMessage();
        populateWSM(sdr);
        sdr->setKind(selfR);
        sdr->setWsmData(to_string(vehicleId).c_str());
        scheduleAt(current_task_time, sdr->dup());
        bp_list[vehicleId] = onD;
    }
}

void TaskOffload::sendResult(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    int vehicleId;
    ss >> vehicleId;
    if(!on_data_check(wsm, vehicleId)) return;

    formal_out(("sending result to " + to_string(vehicleId)).c_str(), 2);
    task myTask = work_info.at(vehicleId);
    send_data(myTask.result_size, vehicleId, 0);        // ensure reliability
    work_info.erase(vehicleId);
    bp_list[vehicleId] = selfR;
}

void TaskOffload::sendDup(WaveShortMessage* wsm)
{
    wsm->setKind(onB);          // only for debug, to send beacon
    sendDown(wsm->dup());    
}

// following are event functions

void TaskOffload::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if(stage == 0)
    {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        node_type = rand() < 0.5*RAND_MAX? TaV:SeV;              // can be initialized by the position or other properties of nodes
        current_task_time = 0;
        //        CPU_freq_max = uniform(2, 6) * 1e9;

        CPU_freq_max = 4e9;                                      // fixed at first
        CPU_percentage = 0.3;
        idle_state = true;
        sig = registerSignal("sig");
    }
    else if(stage == 1)
    {
        formal_out("registering...", 1);
        WaveShortMessage* ini = new WaveShortMessage();
        if(node_type == SeV)
        {       
            Handler[selfB] = &TaskOffload::sendBeacon;
            Handler[onD] = &TaskOffload::processTask;
            Handler[onJ] = &TaskOffload::processBrief;
            Handler[selfR] = &TaskOffload::sendResult;
            Handler[selfDup] = &TaskOffload::sendDup; 
            sendBeacon(ini);
        }
        else
        {           
            Handler[selfG] = &TaskOffload::handleOffload;
            Handler[onB] = &TaskOffload::handleBeacon;
            Handler[onD] = &TaskOffload::updateResult;
            handleOffload(ini);
        }   
    }
}

void TaskOffload::finish() { BaseWaveApplLayer::finish(); }
void TaskOffload::onBSM(BasicSafetyMessage* bsm) {};
void TaskOffload::onWSA(WaveServiceAdvertisment* wsa)
{
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}
void TaskOffload::handlePositionUpdate(cObject* obj)
{
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

void TaskOffload::onWSM(WaveShortMessage* wsm)
{
    if(!checkWSM(wsm)) return;
    if(Handler.find(wsm->getKind()) != Handler.end())
        (this->*Handler[wsm->getKind()])(wsm);
}

void TaskOffload::handleSelfMsg(cMessage* msg) {
    formal_out("Handling self message...", 1);
    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) 
    {
        if(Handler.find(wsm->getKind()) != Handler.end())
            (this->*Handler[wsm->getKind()])(wsm);
        delete(wsm);        
    }
    else BaseWaveApplLayer::handleSelfMsg(msg);
}
