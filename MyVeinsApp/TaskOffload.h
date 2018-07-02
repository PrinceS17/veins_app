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

#ifndef __VEINS_TASKOFFLOAD_H_
#define __VEINS_TASKOFFLOAD_H_

#include <omnetpp.h>
#include <cstdlib>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "ToolFunction.h"         // need definition of enum_type for node_type

using namespace omnetpp;
using namespace std;

//enum ucb_type {ucb, vucb, avucb, rdm};
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
    int total_count = 0;

public:
    void print(bool kind, int vehicleId)
    {
        stringstream ss;
        string str;
        for(int id:SeV_set) ss << id <<" "<< count.at(id) <<"; ";
        if(kind) str = "       Push back: " + to_string(vehicleId) + " ; \n     " + ss.str();
        else str = "       Erase: " + to_string(vehicleId) + " ;\n      " + ss.str();
        EV << str.c_str() << endl;
    }
    void push_back(int vehicleId, ucb_type cur_ucb)
    {
        SeV_set.push_back(vehicleId);
        bit_delay[vehicleId] = 0;
        count[vehicleId] = 0;
        occur_time[vehicleId] = -1;
        last_time[vehicleId] = simTime().dbl();
        if(cur_ucb == ucb || cur_ucb == aucb) // || cur_ucb == vucb)       // only for test
            for(int id:SeV_set)
            {
                bit_delay[id] = 0;
                count[id] = 0;
                occur_time[id] = simTime().dbl();
                last_time[id] = simTime().dbl();
            }
        print(true, vehicleId);
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
        print(false, vehicleId);
        return true;
    }
    void init(int vehicleId, double delay, double x_t, ucb_type cur_ucb)
    {
        if(cur_ucb == avucb || cur_ucb == aucb) bit_delay[vehicleId] = delay / x_t;     // only for test!
        else bit_delay[vehicleId] = delay;
        count[vehicleId] = 1;
        if(cur_ucb == avucb || cur_ucb == vucb) occur_time[vehicleId] = floor(simTime().dbl());         // make tn an interger to avoid t - tn < 1
        last_time[vehicleId] = simTime().dbl();
        total_count ++;
    }
    bool if_exist(int vehicleId)
    {
        return find(SeV_set.begin(), SeV_set.end(), vehicleId) != SeV_set.end();
    }
    bool if_connect(int vehicleId)
    {
        return (count[vehicleId] > 0);
    }
    void update(int vehicleId, double delay, double x_t, ucb_type cur_ucb)
    {
        if(cur_ucb == avucb || cur_ucb == aucb)
            bit_delay[vehicleId] = (bit_delay[vehicleId] * (double)count[vehicleId] + delay / x_t) / ((double)count[vehicleId] + 1);
        else bit_delay[vehicleId] = (bit_delay[vehicleId] * (double)count[vehicleId] + delay ) / ((double)count[vehicleId] + 1);
        count[vehicleId] ++;
        last_time[vehicleId] = simTime().dbl();
        total_count ++;
    }
    void check(map<int, task> whitelist)        // protect id being processed from erasion
    {
        vector<int> temp_set(SeV_set);          // avoid the condition erased by the loop!
        for(auto id:temp_set)
        {
            if(simTime() - last_time.at(id) > 10 && whitelist.find(id) == whitelist.end())
                if(!erase(id)) EV <<"       Error: SeV_set erase failed - no ID: " << id << "!"<< endl;
        }
    }
};

class TaskOffload: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void finish();

public:
    double bc_interval = 1;     // 1 s
    double task_interval = 1;   // periodically generate here
    double Crange = 300;        // communication range
    double speed_limit = 15;
    double ug_speed_limit = 40; // delta speed for uav and vehicle
    double time_limit = 2;      // time limit for work_info chck
    double delay_limit;
    double x_low;               // x- for x_t normalization
    double x_high;
    int serial_max = 2;         // only 2-hop communication is allowed
    int num_rng = 20;           // as num-rngs in omnetpp.ini, 20 currently
    int intime_count = 0;       // # tasks within delay limit 
    int task_count = 0;
    
    // statistic results
    simtime_t job_delay;
    double reliability;         // # success tasks / # task count
    
    // input parameter, initial value, may vary to simulate
    double alpha0 = 0.05;       // y_t/x_t
    double w0 = 1000;           // typical value
    double beta = 1;            // parameter for UCB
    double scale = -1;          // scale for x_t

    //    void formal_out(const char* str, int lv);
    WaveShortMessage* setWsm(int kind, string data, int rcvId, int serial);
    void pos_spd();             // display current speed and position
    void display_SeV();         // display info of SeV
    int nextKind(int kind);     // find the right kind for on_data_check()
    double calculate_scale(vector<task> vt);   // based on the first 10 result

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

    // my own definitions
    enum_type node_type;
    state_type cur_state;           // Mealy machine not work for multi-TaV, replaced by bp_list
    ucb_type cur_ucb;
    string file_name;               // output name
    string external_id;             // the sumo id
    simtime_t current_task_time;
    double x_av;                    // for x_t,
    double dx;                      // x_av - x_min
    double CPU_freq_max;            // [2, 6] GHz
    double CPU_percentage;          // [0.2, 0.5]
    bool idle_state;
    bool ifDataAv;                  // if cur_ucb == avucb || cur_ucb == aucb
    SeV_class SeV_info;
    vector<task> task_vector;       // tasks recorded at each end
    map<int, task> work_info;       // store from brief, to process data
    map<int, int> bp_list;          // block-pass list: block the too old, pass the next, see on_data_check()
    map<int, fType> Handler;
    
    // signal for results
    simsignal_t sig;                // signal for job delay
    simsignal_t sig_r;              // signal for reliability
    
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
    virtual void SeV_work_check();
    virtual bool on_data_check(WaveShortMessage*wsm, int srcId);
    virtual bool checkWSM(WaveShortMessage* wsm);
    virtual int part_choose(vector<double> a);
    virtual int scheduling(double beta, double x_t);
    virtual void local_process(task myTask);
    virtual void send_data(task mytask, int rcvId, int serial);
    virtual void send_data(double size, int rcvId, int serial);

};

#endif
