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

#ifndef __VEINS_TASKOFFLOAD_H_
#define __VEINS_TASKOFFLOAD_H_

#include <omnetpp.h>
#include <cstdlib>
#include <map>
#include <vector>
#include <algorithm>
#include <sstream>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using namespace omnetpp;
using namespace std;

enum enum_type {TaV, SeV};
enum state_type {PB, PT, SR, HO, UR};                     // state name is short for the function it should trigger, e.g. PB -> processBrief

class TaskOffload;
typedef void (TaskOffload::*fType)(WaveShortMessage *);
//typedef pair<int, state_type> tl;                       // tl(traffic light): time(int) & color(state)

struct task
{
    double data_size;          // x_t, [0.2, 1] Mbit
    double result_size;        // y_t
    double cycle_per_bit;      // w_t
    double start;
    double delay;
};

class SeV_class
{
public:   
    vector<int> SeV_set;
    map<int, double> bit_delay;       // u(t, n), n is vehicleId
    map<int, int> count;              // k(t, n)
    map<int, double> occur_time;      // t_n: occurrence time
    map<int, double> last_time;       // time of the last check
    
public:
    void push_back(int vehicleId)
    {
        SeV_set.push_back(vehicleId);
        bit_delay[vehicleId] = 0;
        count[vehicleId] = 0;
        occur_time[vehicleId] = -1;
        last_time[vehicleId] = simTime().dbl();
    }
    bool erase(int vehicleId)
    {
        vector<int>::iterator ite = find(SeV_set.begin(), SeV_set.end(), vehicleId);
        if(ite == SeV_set.end()) return false;
        SeV_set.erase(ite);
        bit_delay.erase(vehicleId);
        count.erase(vehicleId);
        occur_time.erase(vehicleId);
        last_time.erase(vehicleId);
        return true;
    }
    void init(int vehicleId, double delay, double x_t)
    {
        bit_delay[vehicleId] = delay / x_t;
        count[vehicleId] = 1;
        occur_time[vehicleId] = floor(simTime().dbl());         // make tn an interger to avoid t - tn < 1
        last_time[vehicleId] = simTime().dbl();
    }
    bool if_exist(int vehicleId)
    {
        return find(SeV_set.begin(), SeV_set.end(), vehicleId) != SeV_set.end();
    }
    bool if_connect(int vehicleId)
    {
        return (count[vehicleId] > 0);
    }
    void update(int vehicleId, double delay, double x_t)
    {
        bit_delay[vehicleId] = (bit_delay[vehicleId] * count[vehicleId] + delay / x_t) / (count[vehicleId] + 1);
        count[vehicleId] ++;
        last_time[vehicleId] = simTime().dbl();
    }
    void check()
    {
        for(auto id:SeV_set)
        {
            if(simTime() - last_time[id] > 2)
                if(!erase(id)) EV <<"       Error: SeV_set erase failed!";
        }
    }
};

class TaskOffload: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void finish();
    
public:
    double mean_size = 6e5;     // for x_t,
    double bc_interval = 1;     // 1 s
    double task_interval = 1;   // periodically generate here
    double Crange = 200;        // communication range
    double speed_limit = 15;
    simtime_t job_delay;
    
    // input parameter, initial value, may vary to simulate
    double alpha0 = 0.05;       // y_t/x_t
    double w0 = 1000;           // typical value
    double beta = 2;            // parameter for UCB
    double scale = -1;          // scale for x_t
    
    void formal_out(const char* str, int lv);
    void display_SeV();         // display info of SeV
    int nextKind(int kind);     // find the right kind for on_data_check()
    
protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

    // my own definitions
    enum_type node_type;
    state_type cur_state;           // Mealy machine not work for multi-TaV, replaced by bp_list
    simsignal_t sig;
    simtime_t current_task_time;
    double CPU_freq_max;            // [2, 6] GHz
    double CPU_percentage;          // [0.2, 0.5]
    bool idle_state;
    SeV_class SeV_info;
    vector<task> task_vector;       // tasks recorded
    map<int, task> work_info;       // store from brief, to process data
    map<int, int> bp_list;          // block-pass list: block the too old, pass the next, see on_data_check()
    map<int, fType> Handler;
    
protected:
    virtual void onBSM(BasicSafetyMessage* bsm);
    virtual void onWSM(WaveShortMessage* wsm);
    virtual void onWSA(WaveServiceAdvertisment* wsa);
    virtual void handleSelfMsg(cMessage* msg);
    virtual void handlePositionUpdate(cObject* obj);

    // handler for both
    virtual void handleTraffic(WaveShortMessage* wsm);
    
    // handler for TaV
    virtual void handleBeacon(WaveShortMessage* wsm);
    virtual void handleOffload(WaveShortMessage* wsm);      // containing generation part
    virtual void updateResult(WaveShortMessage* wsm);
    
    // handler for SeV
    virtual void sendBeacon(WaveShortMessage* wsm);
    virtual void processBrief(WaveShortMessage* wsm);
    virtual void processTask(WaveShortMessage* wsm);
    virtual void sendResult(WaveShortMessage* wsm);
    virtual void sendDup(WaveShortMessage* wsm);
    
    // tool function
    virtual void relay(WaveShortMessage* wsm);
    virtual bool on_data_check(WaveShortMessage*wsm, int srcId);
    virtual bool checkWSM(WaveShortMessage* wsm);
    virtual int scheduling(double beta, double x_t);
    virtual void local_process(task myTask);
    virtual void send_data(task mytask, int rcvId, int serial);
    virtual void send_data(double size, int rcvId, int serial);
    
};

#endif
