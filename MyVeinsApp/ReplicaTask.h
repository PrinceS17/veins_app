#ifndef __VEINS_REPLICATASK_H_
#define __VEINS_REPLICATASK_H_

#include <omnetpp.h>
#include <cstdlib>
#include <numeric>      // SeV_class:: average_delay:: accumulate
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "ToolFunction.h"

using namespace omnetpp;
using namespace std;

//enum enum_type {TaV, SeV};

class ReplicaTask;
typedef void (ReplicaTask::*fType)(WaveShortMessage *);

struct task
{
    double data_size;
    double result_size;
    double cycle_per_bit;
    double start;
    double delay;
};

class Pid
{
public:
    int id;
    double time;

public:
    Pid(int id = 0, double time = 0) {this->id = id; this -> time = time;}
    Pid(stringstream& ss) { ss >> id >> time; }
    void write(stringstream& ss) { ss << id <<' '<< time <<' '; }
    Pid transform(int vid) { return Pid(vid, time); }
    bool operator < (const Pid& pid)const { return (time < pid.time || (time == pid.time && id < pid.id)); }     // use time as the comparable basis
    bool operator == (const Pid& pid) const { return (time == pid.time && id == pid.id); }  // for find?
};

class SeV_class
{
public:
    vector<int> SeV_set;
    map<int, vector<double> > F;    // CDF of delay of SeVs
    map<int, int> count;
    map<int, double> occur_time;
    map<int, double> last_time;
    int m = 10;                     // # intervals in [0, 1], 10 by default, set from outside
    int total_count = 0;            // actually count of success

public:
    void printF(int id)
    {
        EV << "     ";
        for(auto x:F.at(id)) EV << x << ' ';
        EV << endl;
    }
    double average_delay(int id)
    {
        double sum = 0;
        for(int i = 0; i < m; i ++)
            if(!i) sum += F.at(id).at(i) * ((double)i + 0.5) / (double)m;
            else sum += (F.at(id).at(i) - F.at(id).at(i - 1)) * ((double)i + 0.5) / (double)m;
        return sum;
    }
    bool if_exist(int id)
    {
        return find(SeV_set.begin(), SeV_set.end(), id) != SeV_set.end();
    }
    bool if_connect(int id)
    {
        return (count[id] > 0);
    }
    bool erase(int id)
    {
        vector<int>::iterator ite = find(SeV_set.begin(), SeV_set.end(), id);
        if(ite == SeV_set.end()) return false;
        SeV_set.erase(ite);
        F.erase(id);
        count.erase(id);
        occur_time.erase(id);
        last_time.erase(id);
        return true;
    }
    void push_back(int id)
    {
        SeV_set.push_back(id);
        F[id] = vector<double>(m, 0);
        F[id][m - 1] = 1;
        count[id] = 0;
        occur_time[id] = simTime().dbl();   // modified: occur as push back
        last_time[id] = simTime().dbl();    // not for test
    }
    void update(int id, double delay)
    {
        int ki = floor(delay * m);          // from ki F changes;  m samples in all: 1/m, 2/m, ..., 1
        for(int i = 0; i < m; i ++)
        {
            if(i < ki) F[id][i] *= (double)count[id] / ((double)count[id] + 1);
            else F[id][i] = (F[id][i] * (double)count[id] + 1) / ((double)count[id] + 1);
        }
        count[id] ++;
        last_time[id] = simTime().dbl();
        total_count ++;
    }
    void reset()
    {
        vector<int> temp_set(SeV_set);
        for(int id:temp_set)
        {
            erase(id);
            push_back(id);
        }
    }
    void check(vector<int> white_list)        // hard to get the map from id to task!
    {
        vector<int> temp_set(SeV_set);
        for(int id:temp_set)
        {
            if(simTime() - last_time.at(id) > 10 && find(white_list.begin(), white_list.end(), id) == white_list.end())
                if(!erase(id)) EV <<"       Error: SeV set erase failed, no this ID: " << id << "!" << endl;
        }
    }
};

class ReplicaTask: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void finish();

public:
    double bc_interval = 1;
    double task_interval = 1;
    double Crange = 300;
    double speed_limit = 15;
    double ug_speed_limit = 40;
    double time_limit = 2;      // over it the task in work_info will be erased
    double delay_limit;
    int serial_max = 2;         // preserve the old
    int num_rng = 20;
    int intime_count = 0;
    int task_count = 0;         // count tasks generated
    simtime_t job_delay;
    double reliability;

    double alpha0 = 0.05;
    double w0 = 1000;
    double beta = 1;
    double scale = -1;          // scale for delay to make it distributed in [0, 1]

    WaveShortMessage* setWsm(int kind, string data, int rcvId, int serial);     // only kind is necessary
    void display_SeV();
    void display_bp_list();
    double calculate_scale(vector<task> vt);

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

    // my own definitions
    enum_type node_type;
    simsignal_t sig;
    simsignal_t sig_r;
    string file_name;           // output name
    string external_id;         // the sumo id
    simtime_t current_task_time;
    int K;                      // # replica, parameter
    double x_av;
    double dx;
    double CPU_freq_max;        // [2, 6] GHz
    double CPU_percentage;      // [0.2, 0.5]
    bool idle_state;
    SeV_class SeV_info;
    vector<task> task_vector;   // push back all the tasks, same as before
    vector<double> ttime;   // collect all the task time for search
    map<Pid, task> work_info;   // store tasks by vehicle ID and time
    map<Pid, int> bp_list;      // machine identified by tasks
    map<double, bool> ifFirst;  // <time, bool>: mark whether task is firstly updated
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
    virtual void handleOffload(WaveShortMessage* wsm);
    virtual void sendDataDup(WaveShortMessage* wsm);
    virtual void updateResult(WaveShortMessage* wsm);

    // handler for SeV
    virtual void sendBeacon(WaveShortMessage* wsm);
    virtual void processBrief(WaveShortMessage* wsm);
    virtual void processTask(WaveShortMessage* wsm);
    virtual void sendResult(WaveShortMessage* wsm);
    virtual void sendDup(WaveShortMessage* wsm);

    // tool function using ReplicaTask variables
    virtual void SeV_work_check();          // erase old/failed entry in work_info and SeV set before offloading
    virtual bool on_data_check(WaveShortMessage* wsm, Pid pid);
    virtual bool checkWSM(WaveShortMessage* wsm);
    virtual vector<int> scheduling(double beta, double x_t);                // data size actually not needed in replica
    virtual double processor(task myTask);                            // sort out the processor part: queuing method
    virtual void local_process(task myTask);
    virtual void send_data(task mytask, int rcvId, int serial);
    virtual void send_data(Pid pid, double size, int rcvId, int serial);     // though may not all needed, preserve to keep the same realization

};

#endif
