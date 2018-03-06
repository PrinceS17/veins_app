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

#ifndef __VEINS_APPOFHANDLER_H_
#define __VEINS_APPOFHANDLER_H_

#include <omnetpp.h>
#include <cstdlib>
#include <map>
#include <vector>
#include <algorithm>
#include <sstream>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using namespace omnetpp;
using namespace std;

/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

typedef void (*fType)(WaveShortMessage *);                      // def fType for handler map


// this is a type of node
enum enum_type {PROCESSOR, REQUESTER};
enum state_type {ST, BEA, CAC, DISQ, DISP, SCH, DAT};           // could be same for R & P

int max_num_neighbor = 50;              // follow AVE paper
double phi = 0.8;                       // for NAI calculation

struct CmpByValue
{
    bool operator()(const pair<int, double> p1, const pair<int, double> p2)
    {
        return p1.second < p2.second;
    }
    
}cmp;

struct job
{
    int data_size;          // TX data
    int result_size;        // after processing
    double workload;        // for processing time calculation
    double utility;
    map<int, double> bid;   // only in requester: multiple processors and their bids; it is cumbersome for processor to keep it
    
    double start;           // start time of this job
    simtime_t delay;        // the whole delay of this job, containing communication and computation time
};

struct NAI
{
    //int vehicleId;
    bool ifIdle;
    int hopNum;
    simtime_t expiredTime;

};

class NAI_table
{
public:
    map<int, NAI> NAI_map;          // map of vehicle ID and the value
    vector<int> NAI_vector;         // vector of vehicle ID for search
    int length;
    double value;

public:
    NAI_table() {length = 0;}
    //~NAI_table();

    void push_back(int vehicleId, NAI entry)       // push bask the entry to the end of NAI table
    {
        NAI_map.insert(pair<int, NAI>(vehicleId, entry));
        NAI_vector.push_back(vehicleId);
        length ++;
    }

    void push_back(int vehicleId, bool ifIdle, int hopNum, simtime_t expiredTime)
    {
        NAI entry = {ifIdle, hopNum, expiredTime};
        this->push_back(vehicleId, entry);

    }

    bool erase(int vehicleId)
    {
        if(NAI_map.find(vehicleId) != NAI_map.end())        // ID exists in table
        {
            NAI_map.erase(vehicleId);                       // suppose that must be the 1st entry to be removed because it's earliest
            // NAI_vector.pop_back();                          // otherwise we have to look through all entries: may cause problem, yes it caused...
            NAI_vector.erase(std::find(NAI_vector.begin(), NAI_vector.end(), vehicleId));
            length --;
            return true;
        }
        else return false;
    }

    bool find(int vehicleId)
    {
        return (NAI_map.find(vehicleId) != NAI_map.end()? true:false);
    }

    vector<int> generate_hop1()
    {
        vector<int> hop1_neighbor;
        for(int i = 0; i < length; i ++)
        {
            if(NAI_map[NAI_vector.at(i)].hopNum < 2)
                hop1_neighbor.push_back(NAI_vector.at(i));
        }
        if(hop1_neighbor.size() <= max_num_neighbor)
            return hop1_neighbor;
        else
        {
            vector<int> temp(max_num_neighbor);
            for(int i = 0; i < max_num_neighbor; i ++)
            {
                // int ri = randint(0, hop1_neighbor.size() - 1 - i);                                 // get random max_num entries from hop1 vector
                int ri = rand() % static_cast<int>(hop1_neighbor.size() - i);
                temp.push_back(hop1_neighbor.at(ri));
                hop1_neighbor.erase(hop1_neighbor.begin() + ri);
            }

            // temp.assign(hop1_neighbor.end() - max_num_neighbor + 1, hop1_neighbor.end());       // get last max_num entries, here not randomly
            return temp;
        }

    }

    double calculate_NAI()      // O(n)
    {
        double NAI_value = 0;
        for(int i = 0; i < length; i ++)
        {
            if(NAI_map[NAI_vector.at(i)].hopNum == 1)
                NAI_value += phi;
            else if(NAI_map[NAI_vector.at(i)].hopNum == 2)
                NAI_value += phi * phi;
            else ;
        }
        return NAI_value;
    }

    void update()                // delete expire entries and calculate NAI value
    {
        // debug only
        EV <<"NAI length = "<< length <<endl;
        for(int i = 0; i < length; i ++)
            EV <<"  No. "<< i <<", id: "<< NAI_vector.at(i)<<" , # hop: "<< NAI_map[NAI_vector.at(i)].hopNum << endl;  
        for(int i = 0; i < length; i ++)
        {
            if(NAI_map[NAI_vector.at(i)].expiredTime <= simTime())
                erase(NAI_vector.at(i));
        }
        value = calculate_NAI();            
    }

};

class AppOfHandler : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void finish();
    protected:
        simtime_t lastDroveAt;
        bool sentMessage;
        int currentSubscribedServiceId;
        
        // my own definitions
        enum_type node_type;            // the node is requester or processor
        map<int,int> data_size;    // record the change of task sizes for both requester and processor
        map<int, job> work_info;   // record the task queue
        queue<job> job_queue;      // imitate the processing queue
        vector<job> job_vector;    // store necessary information of jobs to be sent
        simtime_t current_task_time;
        double computing_speed;         // unified, U(1,2) for workload
        bool idleState;                 // another name of ifIdle, in requester: false at data transmission; in processor: false when processing
        NAI_table naiTable;
        state_type current_state;        // state like bea, cac...
        
        // my handler map
        map<int, fType> Handler;
        
        // my statistic
        //cOutVector delayVec;
        simsignal_t sig;

    protected:
        virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);
        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);
        
        // handler for both
        virtual void handleBeacon(WaveShortMessage* wsm);       // as bea(wsm, ss);
        
        // handler for traffic info before
        virtual void handleTraffic(WaveShortMessage* wsm);      // original code in 'T'
        
        // handler for requester
        virtual void generateJob(WaveShortMessage* wsm);        // as cac(), contains generate_job(), doesn't need wsm; no need to register
        virtual void handleCache(WaveShortMessage* wsm);        // GENERATE_JOB_EVT part, doesn't need wsm
        virtual void handleDISQ(WaveShortMessage* wsm);         // as dis(R:0), contains EREQ; doesn't need wsm; no need to register
        virtual void checkEREQ(WaveShortMessage* wsm);          // CHECK_EREQ_EVT part, no need to rewrite local_process()
        virtual void handleDISP(WaveShortMessage* wsm);         // as dis(R:1), contains sch(), rewriting needed: no dat() part, send self-msg containing scheduling result at now or future
        virtual void sendData(WaveShortMessage* wsm);           // ENTER_SCH_EVT part and dat(R:0), also contains SEND_DATA_EVT part (or another handler is needed...)
        virtual void handleResultData(WaveShortMessage* wsm);   // as dat(R:1)
        
        // handler for processor
        virtual void sendBeacon(WaveShortMessage* wsm);         // as bea(wsm, ss = NULL), same as SEND_MY_BEACON_EVT, contains send_beacon(); doesn't need wsm
        virtual void processEREQ(WaveShortMessage* wsm);        // as dis(P:0), contains EREP
        virtual void processJobBrief(WaveShortMessage* wsm);    // as dat(P:0)
        virtual void processData(WaveShortMessage* wsm);        // as dat(P:1)
        
        // tool functions
        virtual void checkWSM(WaveShortMessage* wsm);           // do things at the start of onWSM
        virtual vector<int> scheduling(vector<int> v0, int type);       // retain for more complex algorithm
        virtual void send_data(int size, int rcvId, int serial, simtime_t time);
        virtual void send_data(job myJob, int rcvId, int serial, simtime_t time);
        virtual void local_process(queue<job> job_queue);
        virtual void local_process(vector<job> job_vec);
        virtual void relay(WaveShortMessage* wsm);
        virtual bool on_data_check(WaveShortMessage* wsm, int srcId);
        
    };

#endif
