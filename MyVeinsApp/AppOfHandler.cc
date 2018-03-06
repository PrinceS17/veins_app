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
#include "AppOfHandler.h"
using namespace std;

// kind for self-message
#define SEND_DATA_EVT 2
#define SEND_MY_BC_EVT 3
#define GENERATE_JOB_EVT 4
#define CHECK_EREQ_EVT 5
#define ENTER_DAT_EVT 6

// kind for received message
#define onB 7       // R & P: my beacon
#define onQ 8       // P: EREQ
#define onP 9       // R: EREP
#define onJ 10      // P: job brief
#define onD 11      // R & P: data
#define onT 12      // R & P: traffic info

Define_Module(AppOfHandler);

// job parameters
double job_time = 0;
double lambda = 0.2;
double dataSize = 100e3;
double resultSize = 100e3;
double work_mean = 4;
double slot = 0.001;         // what is the typical value? 

bool TxEnd;

double rtd = 0.05;           // 50 ms
double t_disc = 0;
double speedLimit = 15;      // 15 m/s
double my_bc_interval = 1;   // 1 s, same as the BSM period
double ereq_interval = 0.02; // compared to rtd

bool ifFirst = true;
double t_end;                // for end of caching

// global variable
simtime_t job_delay;
stringstream ss_null("");

// Poission Process Setting
default_random_engine job_g, work_g;
exponential_distribution<double> jobInterval_dstrb(lambda);         // intervals between jobs
exponential_distribution<double> workload_dstrb(1 / work_mean);     // job workload

void formal_out(const char * str, int lv)   // use output to debug
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


void AppOfHandler::checkWSM(WaveShortMessage* wsm)
{
    if(strlen(wsm->getWsmData()) < 50)
        formal_out(wsm->getWsmData(), 1);
    else
    {
        string temp;
        temp.assign(wsm->getWsmData(), wsm->getWsmData() + 50);
        formal_out((temp + "... for short").c_str(), 1);
    }

    // discard 3-hop message or self message
    if(wsm->getSerial() >= 3) 
    {
        formal_out("Discard 3-hop message!", 1);
        return;
    }

    if(NULL == wsm->getWsmData()) {formal_out("Error: void pointer!", 1); return;}              // check if void pointer
    if(strlen(wsm->getWsmData()) <= 1) {formal_out("Error: receive too little data!", 1); return;}       // check data length        
}

void AppOfHandler::handleTraffic(WaveShortMessage* wsm)
{
    formal_out("traffic info...", 2);
    findHost()->getDisplayString().updateWith("r=16,green");
    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wdata, 9999);
    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + 10*slot, wsm->dup());
    }
}

void AppOfHandler::initialize(int stage) {
    formal_out("Initializing...", 1);

    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << endl;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        // srand((unsigned)time(0));
        node_type = rand() < 0.5*RAND_MAX? REQUESTER:PROCESSOR;              // can be initialized by the position or other properties of nodes

        current_task_time = 0;
        computing_speed = uniform(1,2);                                      // can be specified by our demand
        idleState = true;
        TxEnd = true;
        naiTable.push_back(myId, idleState, 0, simTime() +  1000 * my_bc_interval);       // my ID should never expired? 200 correctly
        current_state = ST;
        sig = registerSignal("sig");

    }
    else if (stage == 1) {
        //Initializing members that require initialize other modules goes here
        vector<int> initialId(1,myId);
        if(node_type == PROCESSOR)
        {
            Handler[onB] = handleBeacon;
            Handler[onT] = handleTraffic;

            Handler[SEND_MY_BC_EVT] = sendBeacon;  
            Handler[onQ] = processEREQ;
            Handler[onJ] = processJobBrief;
            Handler[onD] = processData;
            send_beacon(initialId);          // cannot get myid? need some other operation?
        }
        else
        {
            Handler[onB] = handleBeacon;
            Handler[onT] = handleTraffic;

            Handler[GENERATE_JOB_EVT] = handleCache;
            Handler[CHECK_EREQ_EVT] = checkEREQ;
            Handler[onP] = handleDISP;
            Handler[ENTER_DAT_EVT] = sendData;
            Handler[onD] = handleResultData;

            generate_job(lambda, 1, 1, 0);   // virtual job, to starting normal job caching
            t_disc = simTime().dbl();
        }
    }
}

void AppOfHandler::onWSM(WaveShortMessage* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples, 

    // actaully begin decoding, should be in handler
/*    const char* wdata = string(wsm->getWsmData()).c_str();
    stringstream ss;
    ss << wsm->getWsmData();
*/
    
    if(Handler.find(wsm->getKind()) != Handler.end())       // considering node type
        Handler[wsm->getKind()](wsm);

}




