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
#include "MyVeinsApp.h"
#define SEND_DATA_EVT 2

Define_Module(MyVeinsApp);

void MyVeinsApp::send_beacon(int vehicleId, Coord speed, bool ifIdle, std::vector<int> hop1_Neighbor)         // send once, print all the information into one string of wsm data
{
    // force converting each parameter into string for test now

    string str = 'B' + (string)vehicleId + (string)speed.x + (string)speed.y + (string)speed.z + (string)ifIdle;      // string may have some offload?
    for(int i = 0; i < hop1_Neighbor.size(); i ++)
        str += (string)hop1_Neighbor.at(i);

    // send beacon now
    WaveShortMessage * bc = WaveShortMessage();
    populateWSM(bc);
    bc->setWsmData(str.c_str());
    scheduleAt(simTime(), bc);

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
        data->setKind(SEND_DATA_EVT);        // cooperate with handleSelfMsg
        data->setWsmLength(l);
        data->setWsmData(str.c_str());
        scheduleAt(time + slot, data);   // send each wsm in every slot, also could be other method

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
        computing_speed = 1e3;

    }
    else if (stage == 1) {
        //Initializing members that require initialized other modules goes here
        send_beacon(myid, curSpeed, true, NULL);          // cannot get myid? need some other operation?

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
    wdata = wsm->getWsmData() + 1;                                      //

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


    case 'B':   // AVE beacon
        if(node_type == PROCESSOR) break;           // only requesters care about AVE beacon
        string str(wdata);

        // decode the information of beacon
        int vehicleId = str.substr(0, sizeof(int)).c_str(); str = str.substr(sizeof(int));
        double vx = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        double vy = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        double vz = str.substr(0, sizeof(double)).c_str(); str = str.substr(sizeof(double));
        bool ifIdle = str.substr(0, sizeof(bool)).c_str(); str = str.substr(sizeof(bool));

        // decode the neighbor ID




        // update the NAI table







        ;


    case 'D':   // data
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
                process_queue.push_back(this_size);                                                                    // push the data into processor queue, maybe not useful now
                current_task_time = std::max(simTime(), current_task_time) + (double)this_size/computing_speed;        // this setting considers the delay of processing

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

    // example in TraCIDemo means th at change channel according to wsa and not use more information
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

    // AVE beacon
//    if(msg->getKind() == SEND_WSA_EVT){
//       WaveServiceAdvertisment* wsa = new WaveServiceAdvertisment();
//       populateWSM(wsa);
//       sendDown(wsa);
//       cMessage *bc_msg = new cMessage("new beacon evt", 1);
//       scheduleAt(simTime() + 2, bc_msg);
//       EV<<"Sending beacon periodically!\n\n\n\n\n";
//   }



    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {         // if the pointer exists or not?
        if(msg->getKind() == SEND_DATA_EVT)                                     // send data continuously
        {
            //not send the wsm for several times like traffic update
            sendDown(wsm->dup());                 // should consider resend or not?
            delete(wsm);
        }
        else
        {
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
