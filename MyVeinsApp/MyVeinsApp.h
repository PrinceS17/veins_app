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

#ifndef __VEINS_MYVEINSAPP_H_
#define __VEINS_MYVEINSAPP_H_

#include <omnetpp.h>
#include <map>
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

// this is a type of node
enum enum_type {PROCESSOR, REQUESTER};
int max_num_neighbor = 50;              // follow AVE paper

// struct of NAI entry
typedef struct NAI
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

public:
    NAI_table() {length = 0;}
    ~NAI_table();

    void push_back(int vehicleId, NAI entry)       // push bask the entry to the end of NAI table
    {
        NAI_map.insert(pair<int, NAI>(vehicleId, entry));
        NAI_vector.push_back(vehicleId);
        length ++;
    }

    void push_back(int vehicleId, bool ifIdle, int hopNum, simTime expiredTime)
    {
        NAI entry = {ifIdle, hopNum, expiredTime};
        this->push_back(vehicleId, entry);

    }

    bool erase(int vehicleId)
    {
        if(NAI_map.find(vehicleId) != NAI_map.end())        // ID exists in table
        {
            NAI_map.erase(vehicleId);                       // suppose that must be the 1st entry to be removed because it's earliest
            NAI_vector.pop_back();                          // otherwise we have to look through all entries
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
                int ri = randint(0, hop1_neighbor.size() - 1 - i);                                 // get random max_num entries from hop1 vector
                temp.push_back(hop1_neighbor.at(ri));
                hop1_neighbor.erase(ri);
            }

            // temp.assign(hop1_neighbor.end() - max_num_neighbor + 1, hop1_neighbor.end());       // get last max_num entries, here not randomly
            return temp;
        }

    }

}

class MyVeinsApp : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void finish();
    protected:
        simtime_t lastDroveAt;
        bool sentMessage;
        int currentSubscribedServiceId;

        // my own definitions
        enum_type node_type;            // the node is requester or processor
        std::map<int,int> data_size;    // record the change of task sizes for both requester and processor
        std::queue process_queue;       // imitate the processing queue
        simtime_t current_task_time;
        double computing_speed;
        bool idleState;                 // another name of ifIdle
        NAI_table naiTable;

    protected:
        virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);

        // my own function
        virtual void send_beacon(int vehicleId, Coord speed, bool ifIdle, std::vector<int> hop1_Neighbor);
        virtual void send_data(int size, int rcvId, int serial, simTime time);

    };

#endif
