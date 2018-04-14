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

#include <random>
#include <fstream>
#include <time.h>
#include "ReplicaTask.h"
using namespace std;

Define_Module(ReplicaTask);

// tool functions inside the class

WaveShortMessage* ReplicaTask::setWsm(int kind, string data = "", int rcvId = 0, int serial = 0)
{
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm, rcvId, serial);
    wsm->setKind(kind);
    wsm->setWsmData(data.c_str());
    return wsm;
}

void ReplicaTask::display_SeV()
{
    formal_out("SeV information:", 2);
    formal_out(("scale = " + to_string(scale)).c_str(), 2);
    formal_out("id  average delay       count       occur time      last time", 3);
    stringstream temp;
    for(int id:SeV_info.SeV_set)
    {
        temp << id <<"      "<< SeV_info.average_delay(id) <<"     "<< SeV_info.count.at(id) <<"        "<< SeV_info.occur_time.at(id) <<"      "<< SeV_info.last_time.at(id);
        formal_out(temp.str().c_str(), 3);
    }
}

double ReplicaTask::calculate_scale(vector<task> vt)
{
    vector<double> del;
    for(auto t:vt) del.push_back(t.delay);
    double temp = *max_element(del.begin(), del.end());
    formal_out((to_string(temp) + " s").c_str(), 2);
    return 1 / temp;
}

void ReplicaTask::SeV_work_check()      // now check for pid!
{
    for(int id:SeV_info.SeV_set)
        for(double t:ttime)
        {
            Pid pid(id, t);
            if(work_info.find(pid) != work_info.end() && simTime().dbl() - t > time_limit)
            {
                work_info.erase(pid);
                if(simTime().dbl() - SeV_info.last_time.at(id) > 5) SeV_info.erase(id);
            }
        }
    SeV_info.check();
}

bool ReplicaTask::on_data_check(WaveShortMessage* wsm, Pid pid)
{
    if(wsm->getSenderAddress() != myId)
        if(pid.id == myId || wsm->getRecipientAddress() != myId) return false;   // wrong TX/RX
    int curKind;
    if(bp_list.find(pid) != bp_list.end())
        curKind = bp_list[pid];
    else curKind = 0;
    if(wsm->getKind() != nextKind(curKind, node_type)) return false;            // wrong message kind / machine stage
    return true;
}

bool ReplicaTask::checkWSM(WaveShortMessage* wsm)
{
    if(wsm->getSerial() >= serial_max) return false;
    if(NULL == wsm->getWsmData()) {formal_out("Error: void pointer!", 1); return false;}
    if(strlen(wsm->getWsmData()) <= 1) {formal_out("Error: receive too little data!", 1); return false;}
    return true;
}

vector<int> ReplicaTask::scheduling(double beta, double x_t)        // replica scheduling algorithm: possibly return less than K SeVs!
{
    formal_out("scheduling...", 2);
    vector<int> res;
    int N = SeV_info.SeV_set.size();
    // 0. judge N <= K? if so, choose all SeVs in set [unique in replica scene]
    if(N <= K)
    {
        for(int id:SeV_info.SeV_set) res.push_back(id);
        return res;
    }
    // 2. judge if there's SeV that hasn't been connected -> offload to a subset containing it: should be contained in scheduling
    for(int i = 0; i < N; i ++)
    {
        int id = SeV_info.SeV_set.at(i);
        if(!SeV_info.if_connect(id) || N == 1)
        {
            for(int j = 0; j < N; j ++)     // choose the new SeV and K - 1 SeVs after it
                res.push_back( SeV_info.SeV_set.at((i + j) % N) );
            return res;
        }
    }
    // 3. if all connected, scheduling to get the subset
    map<int, vector<double>> CDF;
    for(int id:SeV_info.SeV_set)
    {
        double d_tn = simTime().dbl() - SeV_info.occur_time.at(id);
        double k = SeV_info.count.at(id);
        for(int i = 0; i < SeV_info.m; i++)
            CDF[id].push_back( 1 - max( 1 - SeV_info.F.at(id).at(i) - sqrt(beta * log(d_tn) / k), 0.0 ));
    }
    res = oracle(SeV_info.SeV_set, CDF, SeV_info.m, K);
    return res;

}

double ReplicaTask::processor(task myTask)
{
    double task_time =  myTask.data_size * myTask.cycle_per_bit / (CPU_freq_max * CPU_percentage);
    if(current_task_time > simTime())
        current_task_time += task_time;
    else
    {
        idle_state = false;
        current_task_time = simTime() + task_time;
    }  
    return task_time;
}

void ReplicaTask::local_process(task myTask)
{
    formal_out("locally process...", 3);
    CPU_percentage = uniform(0.2, 0.5);
    processor(myTask);
    myTask.delay = current_task_time.dbl() - myTask.start;      // record but not emit
}

void ReplicaTask::send_data(Pid pid, double size, int rcvId, int serial)
{
    string str = "sending data of size " + to_string((int)size / 1000) + " Kbit ...";
    formal_out(str.c_str(), 3);                         // preserve the old
    int max_size = 4096;                                // max size of wsm data, except "D" at the start
    int num = ((int)size + 2)/max_size + 1;             // number of wsm, 2 is "ed"
    int last_size = ((int)size + 1)% max_size + 1;      // last wsm size
    for(int i = 0; i < num; i ++)
    {
        int l = max_size;
        if(i == num - 1) l = last_size;
        stringstream ss;
        pid.write(ss);
        for(int j = ss.str().size(); j < l; j ++) ss<< '0';
        if(i == num - 1) ss<< "ed";

        WaveShortMessage* data = setWsm(onD, ss.str(), rcvId, serial);
        if(i == num - 1) 
            for(int i:{1,2,3}) sendDown(data->dup());
        else sendDown(data->dup());                      // commented it can obviously increase the speed
        delete(data);
        if(current_task_time <= simTime()) idle_state = true;
    }
}

void ReplicaTask::send_data(task myTask, int rcvId, int serial)
{
    formal_out("sending task brief...", 3);
    stringstream ss;
    Pid pid(myId, myTask.start);
    pid.write(ss);
    ss << myTask.data_size <<' '<< myTask.result_size <<' '<< myTask.cycle_per_bit <<' '<<myTask.start;
    WaveShortMessage* pre_data = setWsm(onJ, ss.str(), rcvId, serial);
    for(int i:{1,2,3,4,5,6})                            // resend 6 times to ensure the reliability!
        sendDown(pre_data->dup());
    send_data(pid, myTask.data_size, rcvId, serial);
}

// handlers

void ReplicaTask::handleTraffic(WaveShortMessage* wsm)
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
        wsm->setSerial(0);
        sendDown(wsm->dup());       // ? no wait time now
    }
}

void ReplicaTask::handleBeacon(WaveShortMessage* wsm)
{
    formal_out("TaV: my beacon...", 2);
    formal_out(wsm->getWsmData(), 2);
    stringstream ss1;
    ss1 << "my SeV now: ";
    for(int id:SeV_info.SeV_set) ss1 << id <<' ';
    formal_out(ss1.str().c_str(), 2);

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
    else 
    {
        stringstream ss1;
        ss1 << "Not available! distance: " << curPosition.distance(position) << "; speed dif: " << curSpeed.distance(speed);
        formal_out(ss1.str().c_str(), 1);
        if(SeV_info.if_exist(vehicleId)) SeV_info.last_time.at(vehicleId) -= 50;        // make it erased next second by check
    }
}

void ReplicaTask::handleOffload(WaveShortMessage* wsm)
{
    formal_out("TaV: generate tasks...", 2);
    formal_out(("My sumo ID: " + external_id).c_str(), 2);
    double x_t = 6e5;
    double st = simTime().dbl();                            // fix it for all pid
    task myTask = {x_t, x_t * alpha0, w0, st};
    WaveShortMessage* tsk = setWsm(selfG);
    scheduleAt(simTime() + task_interval, tsk);

    formal_out("TaV: begin offloading...", 2);
    SeV_work_check();
    fstream out(file_name.c_str(), ios::app | ios::out);
    // 1. judge if SeV set empty -> local process (not now), scheduling and send data
    if(SeV_info.SeV_set.empty()) 
    { 
        local_process(myTask);
        out << endl; 
        return; 
    }
    vector<int> cur_set = scheduling(beta, x_t);
    stringstream ss1;
    ss1 <<"Choice: ";
    for(int rcvId:cur_set) 
    { 
        send_data(myTask, rcvId, 0); 
        ss1 << rcvId << ' ';
    }
    formal_out(ss1.str().c_str(), 3);
    // 4. write to file, update task_count, ttime, ifFirst, work_info and bp_list
    out << endl << st << "s ";
    for(int id:cur_set) out << id <<" ";
    out << "        ";
    for(int id:SeV_info.SeV_set) out << id <<" ";
    out.close();
    findHost()->getDisplayString().updateWith("r=20,blue");
    task_count ++;
    ttime.push_back(st);
    ifFirst[st] = true;
    for(int id:cur_set)
    {
        Pid pid(id, st);
        work_info[pid] = myTask;
        bp_list[pid] = selfG;
    }
}

void ReplicaTask::updateResult(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    Pid pid(ss);
    if(!on_data_check(wsm, pid)) return;
    ss.seekg(-2, ss.end);
    string temp;
    ss >> temp; 
    if(temp == "ed")
    {
        if(work_info.find(pid) == work_info.end()) { formal_out("Already end receiving this result!", 1); return; }
        // 1. emit delay, update ifFirst when it's the earliest
        job_delay = simTime() - pid.time;
        if(ifFirst.at(pid.time)) 
        {
            emit(sig, job_delay);
            ifFirst.at(pid.time) = false;
        }
        // 2. update task_vector, calculate scale, update SeV_info, 
        task myTask = work_info.at(pid);
        myTask.delay = job_delay.dbl();
        task_vector.push_back(myTask);
        if(SeV_info.total_count < 10) scale = calculate_scale(task_vector);
        if(!SeV_info.if_exist(pid.id)) { formal_out("Error: vehicle doesn't exist in SeV table now!", 1); SeV_info.count.at(pid.id);}   // use count.at throw error
        SeV_info.update(pid.id, job_delay.dbl());
        // 3. output to file and elog
        stringstream ss1;
        ss1 <<"Got result, pid: "<< pid.id <<" "<< pid.time <<"; delay = "<< job_delay.dbl();
        formal_out(ss1.str().c_str(), 1);
        fstream out(file_name.c_str(), ios::app | ios::out);
        out <<"     "<< pid.id <<"      "<< job_delay.dbl() <<"     "<< myTask.data_size / 1e6;
        out.close();
        // 4. update work_info, bp_list
        work_info.erase(pid);
        bp_list.erase(pid);
    }
}

void ReplicaTask::sendBeacon(WaveShortMessage* wsm)
{
    formal_out("SeV: send beacon...", 2);
    formal_out(("My sumo ID: " + external_id + "; CPU max freq: " + to_string(CPU_freq_max / 1e9) + " GHz").c_str(), 2);
    stringstream ss;
    ss << myId <<' '<< idle_state <<' '<< curPosition.x <<' '<< curPosition.y <<' '<< curPosition.z <<' '<< curSpeed.x << ' '<< curSpeed.y <<' '<< curSpeed.z <<' ';
    WaveShortMessage* bc = setWsm(selfDup, ss.str());
    scheduleAt(simTime(), bc->dup());
    bc->setKind(selfB);
    if(string(wsm->getWsmData()) == "start")
        scheduleAt(simTime() + 0.95 * bc_interval, bc->dup());
    else scheduleAt(simTime() + bc_interval + uniform(-0.03, 0.03), bc->dup());         // avoid collision
    CPU_percentage = uniform(0.2, 0.5);
}

void ReplicaTask::processBrief(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    Pid pid(ss);
    if(!on_data_check(wsm, pid)) return;
    formal_out("process brief...", 2);
    findHost()->getDisplayString().updateWith("r=20,green");
    formal_out(ss.str().c_str(), 2);
    formal_out((to_string(simTime().dbl()) + " CPU %: " + to_string(CPU_percentage)).c_str(), 2);
    
    task myTask;
    ss >> myTask.data_size >> myTask.result_size >> myTask.cycle_per_bit >> myTask.start;
    work_info[pid] = myTask;
    bp_list[pid] = onJ;
    
}

void ReplicaTask::processTask(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    Pid pid(ss);
    if(!on_data_check(wsm, pid)) return;
    
    task myTask = work_info.at(pid);
    ss.seekg(-2, ss.end);
    string temp;
    ss >> temp;
    if(temp == "ed")
    {
        double task_time = processor(myTask);
        stringstream ss1;
        ss1 <<"task time: "<< task_time <<"; schedule time: "<< current_task_time.dbl() <<"; task pid: ";
        pid.write(ss1);
        formal_out(ss1.str().c_str(), 3);
        WaveShortMessage* sdr = setWsm(selfR, to_string(pid.id) +" "+ to_string(pid.time));
        scheduleAt(current_task_time, sdr->dup());
        bp_list.at(pid) = onD;
    }
}

void ReplicaTask::sendResult(WaveShortMessage* wsm)
{
    stringstream ss(wsm->getWsmData());
    Pid pid(ss);
    if(!on_data_check(wsm, pid)) return;
    
    stringstream ss1;
    ss1 << "sending result back, pid: ";
    pid.write(ss1);
    formal_out(ss1.str().c_str(), 2);
    task myTask = work_info.at(pid);
    send_data(Pid(myId, pid.time), myTask.result_size, pid.id, 0);
    work_info.erase(pid);
    bp_list.at(pid) = selfR;
}

void ReplicaTask::sendDup(WaveShortMessage* wsm)
{
    findHost()->getDisplayString().updateWith("r=12,yellow");
    wsm->setKind(onB);
    sendDown(wsm->dup());
}

// basic event functions
void ReplicaTask::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if(stage == 0)
    {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        external_id = mobility->getExternalId();
        node_type = (external_id.c_str()[0] - '0') % 4 == 0? TaV:SeV;
        K = par("K").longValue();
        file_name = "offlog_re_sumoid_" + external_id + "_myid_" + to_string(myId) + "_" + to_string(time(NULL));
        current_task_time = 0; 
        x_av = 6e5;
        dx = 0;
        CPU_freq_max = uniform(2, 6, myId % num_rng) * 1e9;
        idle_state = true;
        sig = registerSignal("sig");
    }
    else if(stage == 1)
    {
        WaveShortMessage* ini = new WaveShortMessage();
        ini->setWsmData("start");
        if(node_type == SeV)
        {
            Handler[selfB] = &ReplicaTask::sendBeacon;
            Handler[onD] = &ReplicaTask::processTask;
            Handler[onJ] = &ReplicaTask::processBrief;
            Handler[selfR] = &ReplicaTask::sendResult;
            Handler[selfDup] = &ReplicaTask::sendDup; 
            sendBeacon(ini);
        }
        else
        {
            fstream out(file_name.c_str(), ios::app | ios::out);
            out <<" T  Chosen           Candidates            From    Delay    Data size"<< endl;
            Handler[selfG] = &ReplicaTask::handleOffload;
            Handler[onB] = &ReplicaTask::handleBeacon;
            Handler[onD] = &ReplicaTask::updateResult;
            handleOffload(ini);
        }
    }
}

void ReplicaTask::finish() { BaseWaveApplLayer::finish(); }
void ReplicaTask::onBSM(BasicSafetyMessage* bsm) {};
void ReplicaTask::onWSA(WaveServiceAdvertisment* wsa)
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
void ReplicaTask::handlePositionUpdate(cObject* obj)
{
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    formal_out("Handling position update...", 1);
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

void ReplicaTask::onWSM(WaveShortMessage* wsm)
{
    if(!checkWSM(wsm)) return;
    if(Handler.find(wsm->getKind()) != Handler.end())
        ( this->*Handler.at(wsm->getKind()) )(wsm);
}

void ReplicaTask::handleSelfMsg(cMessage* msg)
{
    formal_out("Handling self message...", 1);
    if(WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg))
    {
        if(Handler.find(wsm->getKind()) != Handler.end())
            ( this->*Handler.at(wsm->getKind()) )(wsm);
        delete(wsm);
    }
    else BaseWaveApplLayer::handleSelfMsg(msg);
}
