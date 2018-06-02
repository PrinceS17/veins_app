//
// Copyright (C) 2018 Jinhui Song <jssong9617@gmail.com>
//
// Documentation for the module is at https://github.com/PrinceS17/veins_app
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
#include <fstream>
#include <time.h>
#include "TaskOffload.h"        // which includes ToolFunction.h
using namespace std;
//
//#define selfB 2
//#define selfG 3
//#define selfDup 4
//#define selfR 5
//#define onB 11
//#define onT 12
//#define onD 13
//#define onJ 14


Define_Module(TaskOffload);

// following are tool functions

void TaskOffload::pos_spd()
{
    stringstream ss1;
    ss1 << "my position: " << curPosition << "; my speed: "<< curSpeed;
    formal_out(ss1.str().c_str(), 3);
}

WaveShortMessage* TaskOffload::setWsm(int kind, string data, int rcvId = 0, int serial = 0)
{
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm, rcvId, serial);
    wsm->setKind(kind);
    wsm->setWsmData(data.c_str());
    return wsm;
}

void TaskOffload::display_SeV()
{
    formal_out("SeV information:", 2);
    string str = cur_ucb == avucb? "scale = " + to_string(scale / 1e6) + " M":
            "scale = " + to_string(scale);
    formal_out(str.c_str(), 2);
    if(cur_ucb == avucb) formal_out("id     delay/bit (us)      count     occur time     last time", 3);
    else formal_out("id     delay (s)        count     occur time     last time", 3);
    string temp;
    for(auto id:SeV_info.SeV_set)
    {
        if(cur_ucb == avucb)     // for test
            temp = to_string(id) + "      " + to_string(SeV_info.bit_delay.at(id) * 1e6) + "              " + to_string(SeV_info.count.at(id)) + "       " + to_string(SeV_info.occur_time.at(id)) + "        " + to_string(SeV_info.last_time.at(id));
        else temp = to_string(id) + "      " + to_string(SeV_info.bit_delay.at(id)) + "              " + to_string(SeV_info.count.at(id)) + "       " + to_string(SeV_info.occur_time.at(id)) + "        " + to_string(SeV_info.last_time.at(id));    
        formal_out(temp.c_str(), 3);
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

double TaskOffload::calculate_scale(vector<task> vt)
{
    vector<double> bitDelay;
    for(auto t:vt)
        if(cur_ucb == avucb) bitDelay.push_back(t.delay / t.data_size);              // only for test!!!
        else bitDelay.push_back(t.delay);
    double temp = *max_element(bitDelay.begin(), bitDelay.end());
    if(cur_ucb == avucb) formal_out((to_string(temp * 1e6) + " us").c_str(), 2);
    else formal_out((to_string(temp) + " s").c_str(), 2);
    return 1 / temp;    // before: 0.96
}


void TaskOffload::relay(WaveShortMessage* wsm)          // suppressed now
{
    wsm->setSenderAddress(myId);
    if(wsm->getSerial() == serial_max - 1) return;      // control old wsm at TX side
    wsm->setSerial(wsm->getSerial() +1);
    sendDown(wsm->dup());
    string str = "Relay! Current serial is " + to_string(wsm->getSerial());
    //    formal_out(str.c_str(), 3);
}

void TaskOffload::SeV_work_check()
{
    for(int id:SeV_info.SeV_set)
        if(work_info.find(id) != work_info.end() && simTime().dbl() - work_info.at(id).start > time_limit)
        {
            work_info.erase(id);
            if(simTime().dbl() - SeV_info.last_time.at(id) > 5) SeV_info.erase(id);
        }
    SeV_info.check(work_info);
}

bool TaskOffload::on_data_check(WaveShortMessage* wsm, int srcId)
{
    if(wsm->getSenderAddress() != myId )                  // it's not a self-msg, avoid printing relay for sending result
    {
        if(srcId == myId) return false;
        else if(wsm->getRecipientAddress() != myId) 
        {
            // relay banned
            //            if(wsm->getKind() == onJ) relay(wsm);         // only relay job brief to reduce the msg amount
            //            else if(wsm->getKind() == onD && uniform(0, 1) > 0.7)
            //                relay(wsm);
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
        return false;
    }
    return true;
}

bool TaskOffload::checkWSM(WaveShortMessage* wsm)
{
    if(wsm->getSerial() >= serial_max) return false;                                                           // discard 3-hop message or self message
    if(NULL == wsm->getWsmData()) {formal_out("Error: void pointer!", 1); return false;}                       // check if void pointer
    if(strlen(wsm->getWsmData()) <= 1) {formal_out("Error: receive too little data!", 1); return false;}       // check data length   
    return true;
}

int TaskOffload::part_choose(vector<double> a)
{
    int id, num = 0, res = -1;
    vector<int> tot;
    for(double x:a)
        if(x != 9999) num++;
        else tot.push_back(num);
    tot.push_back(10000);                                      // for the number after last tree
    id = rand() % num;
    for(int i = 0; i < tot.size(); i ++)
        if(id + 1 <= tot.at(i)) return SeV_info.SeV_set.at(id + i);
}

int TaskOffload::scheduling(double beta, double x_t)           // AVUCB algorithm first
{
    formal_out("scheduling...", 2);
    vector<double> utility;
    for(int i = 0; i < SeV_info.SeV_set.size(); i ++)
    {
        int id = SeV_info.SeV_set.at(i);
        if(work_info.find(id) != work_info.end())
            utility.push_back(9999);    // avoid choosing the SeV which is processing my last job now
        else
        {
            double xt1 = x_t > x_av ? 1.0:0.0;      // 300s: worst                        half explore & half exploit
            //            double xt1 = 0;                         // for same x_t; 200s: avucb between ucb & rdm      all explore
            //            double xt1 = x_t < 4e5 ? 0.0:
            //                    (x_t > 8e5 ? 1.0:               // 300s: avucb even worse than ucb
            //                            2.5 * x_t / 1e6 - 1);   // 200s: avucb same as ucb; .         the primitive expression of avucb 

            double d_tn = simTime().dbl() - SeV_info.occur_time.at(id);     // for UCB, the occur time is the push back time
            double k = SeV_info.count.at(id);
            if(cur_ucb == avucb)
                utility.push_back( SeV_info.bit_delay.at(id) * scale - sqrt(2* beta * (1 - xt1) * log(d_tn) / k) );
            else if(cur_ucb == vucb || cur_ucb == ucb)
                utility.push_back( SeV_info.bit_delay.at(id) * scale - sqrt(beta * log(d_tn) / k) );    // right one for vucb
            else utility.push_back(1);          // otherwise utility has no entry in rdm mode
        }
        string str = "SeV " + to_string(id) + ": u = " + to_string(utility.at(i)) + " ; scaled bit delay = " + to_string(SeV_info.bit_delay.at(id) * scale);
        formal_out(str.c_str(), 3);
    }

    int vid = SeV_info.SeV_set[min_element(utility.begin(), utility.end()) - utility.begin()];
    if(cur_ucb == rdm) vid = part_choose(utility);
    string str = "Choice: " + to_string(vid);
    formal_out(str.c_str(), 3);
    return vid;
}

void TaskOffload::local_process(task myTask)
{
    formal_out("locally process...", 3);
    CPU_percentage = uniform(0.2, 0.5 );
    double task_time =  myTask.data_size * myTask.cycle_per_bit / (CPU_freq_max * CPU_percentage);
    if(current_task_time > simTime())
        current_task_time += task_time;
    else
    {
        idle_state = false;
        current_task_time = simTime() + task_time;
    }    
    myTask.delay = current_task_time.dbl() - myTask.start;        // recored the final delay
}

void TaskOffload::send_data(double size, int rcvId, int serial)
{
    string str = "sending data of size " + to_string((int)size / 1000) + " Kbit ...";
    formal_out(str.c_str(), 3);
    int max_size = 4096;                                // max size of wsm data, except "D" at the start
    int num = ((int)size + 2)/max_size + 1;             // number of wsm, 2 is "ed"
    int last_size = ((int)size + 1)% max_size + 1;      // last wsm size
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
    formal_out(("sending task brief to " + to_string(rcvId) + "...").c_str(), 3);
    stringstream ss;
    ss<< myId <<' '<< myTask.data_size <<' '<< myTask.result_size <<' '<< myTask.cycle_per_bit <<' '<<myTask.start;
    WaveShortMessage* pre_data = new WaveShortMessage();
    populateWSM(pre_data, rcvId, serial);
    pre_data->setKind(onJ);
    pre_data->setWsmData(ss.str().c_str());
    for(int i:{1,2,3,4,5,6})                            // resend 6 times to ensure the reliability!
        sendDown(pre_data->dup());
    send_data(myTask.data_size, rcvId, serial);
}

// following are handlers

void TaskOffload::handleTraffic(WaveShortMessage* wsm)
{
    formal_out("traffic info...", 2);
    if (mobility == NULL) return;               // used for UAV
    const char* wdata = wsm->getWsmData();
    findHost()->getDisplayString().updateWith("r=16,green");
    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wdata, 9999);
    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setKind(onT);
        wsm->setSerial(0);
        sendDown(wsm->dup());       // ? no wait time now
    }
}

void TaskOffload::handleBeacon(WaveShortMessage* wsm)
{

    formal_out("TaV: my beacon...", 2);
    formal_out(wsm->getWsmData(), 2);
    string str = "my SeV now: ";
    for(int id:SeV_info.SeV_set)
        str += to_string(id) + " ";
    formal_out(str.c_str(), 2);

    stringstream ss(wsm->getWsmData());
    int vehicleId;
    Coord position, speed;
    bool ifIdle;
    ss >> vehicleId >> ifIdle >> position.x >> position.y >> position.z >> speed.x >> speed.y >> speed.z;
    bool ifAvailable = (position.z < 10 && curPosition.distance(position) < Crange && curSpeed.distance(speed) < speed_limit)
                                    || ((position.z >= 10 || curPosition.z > 10) && curSpeed.distance(speed) < ug_speed_limit);
    if(SeV_info.if_exist(vehicleId) && ifAvailable)
        SeV_info.last_time.at(vehicleId) = simTime().dbl();  
    else if(!SeV_info.if_exist(vehicleId) && ifAvailable)
        SeV_info.push_back(vehicleId, cur_ucb);
    else 
    {
        stringstream ss1;
        ss1 << "Not available! distance: " << curPosition.distance(position) << "; speed dif: " << curSpeed.distance(speed);
        formal_out(ss1.str().c_str(), 1);
        if(SeV_info.if_exist(vehicleId)) SeV_info.last_time.at(vehicleId) -= 50;        // make it erased next second by check
    }
}

void TaskOffload::handleOffload(WaveShortMessage* wsm)
{

    formal_out("TaV: generate tasks...", 2);
    stringstream ss1;
    ss1 << "My sumo ID: " << external_id <<"; position: "<< curPosition;
    formal_out(ss1.str().c_str(), 2);

    double x_t = 6e5;   // only for test
    //    double x_t = uniform(x_av - dx, x_av + dx, myId % num_rng);
    task myTask = {x_t, x_t * alpha0, w0, simTime().dbl()};
    WaveShortMessage* tsk = new WaveShortMessage();         // schedule next task 
    populateWSM(tsk);
    tsk->setKind(selfG);
    scheduleAt(simTime() + task_interval, tsk);
    formal_out("TaV: begin offloading...", 2);
    int rcvId = -9999;
    SeV_work_check();                                       // for "work info lost" & "dead task & SeV" problem
    fstream out(file_name.c_str(), ios::app | ios::out);    // record the offload process and choice 
    for(int id:SeV_info.SeV_set)
    {
        if(work_info.find(id) != work_info.end()) continue;
        rcvId = id;
    }
    if(SeV_info.SeV_set.empty() || rcvId == -9999)          // if all are busy
    {
        local_process(myTask);
        out << endl; 
        return; 
    }
    rcvId = -9999;
    for(int id:SeV_info.SeV_set)
    {
        if((!SeV_info.if_connect(id) || SeV_info.SeV_set.size() == 1) && work_info.find(id) == work_info.end())   // find one unconnected SeV or only one SeV
        {
            rcvId = id;
            break;
        }
    }   
    if(rcvId == -9999)  rcvId = scheduling(beta, x_t);      // all connected
    out << endl << simTime().dbl() << "s " << rcvId <<"        ";
    for(int id:SeV_info.SeV_set) out << id <<' ';
    out.close();

    send_data(myTask, rcvId, 0);
    findHost()->getDisplayString().updateWith("r=20,blue");
    work_info[rcvId] = myTask;
    task_count ++;
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
        task_vector.push_back(myTask);
        if(SeV_info.total_count <= 10) scale = calculate_scale(task_vector);
        if(!SeV_info.if_exist(vehicleId)) 
            formal_out(" Error: vehicle doesn't exist in SeV table now!", 1);   // following .count.at will throw an out-of-range error
        if(SeV_info.count.at(vehicleId) == 0)
            SeV_info.init(vehicleId, myTask.delay, myTask.data_size, cur_ucb);        // add occurrence time
        else SeV_info.update(vehicleId, myTask.delay, myTask.data_size, cur_ucb);     // maintain u & k
        display_SeV();


        job_delay = myTask.delay;
        if(job_delay < delay_limit) intime_count ++;
        reliability = (double)intime_count / (double)task_count;
        //        reliability = (double)SeV_info.total_count / (double)task_count;
        emit(sig, job_delay);           // send record signal
        emit(sig_r, reliability);

        string str = "Got result from " + to_string(vehicleId) + "; delay = " + to_string(job_delay.dbl()) + "; data size = " + to_string(myTask.data_size / 1e6) + "Mbit";
        formal_out(str.c_str(), 1);
        fstream out(file_name.c_str(), ios::app | ios::out);        // easy for check the reliability      
        out <<"    "<< vehicleId <<"    "<< job_delay.dbl() <<"    "<< myTask.data_size / 1e6;
        out.close();
        work_info.erase(vehicleId);
        bp_list[vehicleId] = onD;
    }
}

void TaskOffload::sendBeacon(WaveShortMessage* wsm)
{
    formal_out("SeV: send beacon...", 2);
    stringstream ss, ss1;
    ss1 << "My sumo ID: "<< external_id <<"; CPU max freq: "<< (CPU_freq_max/1e9) <<" GHz; pos: " << curPosition;
    formal_out(ss1.str().c_str(), 2);
    ss<< myId <<' '<< idle_state <<' '<< curPosition.x <<' '<< curPosition.y <<' '<< curPosition.z <<' '<< curSpeed.x << ' '<< curSpeed.y <<' '<< curSpeed.z <<' ';
    //    WaveShortMessage* bc = new WaveShortMessage();
    //    populateWSM(bc);
    //    bc->setKind(selfDup);
    //    bc->setWsmData(ss.str().c_str());
    //    
    //    WaveShortMessage a(setWsm(selfDup, ss.str()));
    //    WaveShortMessage* bc = a.dup();
    WaveShortMessage *bc = setWsm(selfDup, ss.str());
    scheduleAt(simTime(), bc->dup());
    bc->setKind(selfB);
    if(string(wsm->getWsmData()) == "start") 
        scheduleAt(simTime() + 0.95 * bc_interval, bc->dup());
    else scheduleAt(simTime() + bc_interval + uniform(-0.03, 0.03), bc->dup());         // avoid collision
    CPU_percentage = uniform(0.2, 0.5 );     // randomly change percentage at each period

}

void TaskOffload::processBrief(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    int vehicleId;
    ss >> vehicleId;
    if(!on_data_check(wsm, vehicleId)) return;
    if(curPosition.z > 10) formal_out("process brief: UAV chosen...", 2);
    else formal_out("process brief...", 2);
    findHost()->getDisplayString().updateWith("r=20,green");        // show clearly the chosen SeV
    formal_out(ss.str().c_str(), 2);
    task myTask;
    ss >> myTask.data_size >> myTask.result_size >> myTask.cycle_per_bit >> myTask.start;
    work_info[vehicleId] = myTask;
    stringstream ss1;
    ss1 << simTime().dbl() << " CPU %: " << CPU_percentage ;
    formal_out(ss1.str().c_str(), 2);

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
    send_data(myTask.result_size, vehicleId, 0);            // ensure reliability
    work_info.erase(vehicleId);
    bp_list[vehicleId] = selfR;
}

void TaskOffload::sendDup(WaveShortMessage* wsm)
{
    wsm->setKind(onB);          // to send beacon
    findHost()->getDisplayString().updateWith("r=12,yellow");

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

        //        node_type = rand() < 0.5*RAND_MAX? TaV:SeV;               // can be initialized by the position or other properties of nodes
        //        node_type = (external_id.c_str()[0] - '0') % 2 == 0 ? TaV:SeV;
        if(NULL == mobility) external_id = "40";                            // use UAV as TaV
        else external_id = mobility->getExternalId();
        node_type = (external_id.c_str()[0] - '0') % 4 == 0? TaV:SeV;

        //        node_type = myId/6 % 2 == 0? TaV:SeV;
        delay_limit = par("delay_limit").doubleValue();
        long i = par("cur_ucb").longValue();
        cur_ucb = !i ? ucb : (i == 1 ? vucb : (i == 2? avucb : rdm));
        file_name = "offlog_" + to_string(i) + "_sumoid_" + external_id + "_myId_" + to_string(myId) + "_"+ to_string(time(NULL));  // choose among ucb and TaV
        current_task_time = 0;
        x_av = 6e5;                                              // for x_t,
        dx = 4e5;                                                // x_av - x_min
        CPU_freq_max = uniform(2, 6, myId % num_rng) * 1e9;
        idle_state = true;
        sig = registerSignal("sig");
        sig_r = registerSignal("sig_r");
    }
    else if(stage == 1)
    {
        formal_out("registering...", 1);

        WaveShortMessage* ini = new WaveShortMessage();
        ini->setWsmData("start");
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
            fstream out;
            if(external_id.c_str()[0] == '0') out.open(file_name.c_str(), ios::app | ios::out);
            out <<" T  Chosen      Candidates            From    Delay    Data size"<< endl;

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
    if(mobility == NULL) return;                // for UAV

    // findHost()->getDisplayString().updateWith("r=16,red");
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
