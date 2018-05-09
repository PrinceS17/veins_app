//
// Author: Jinhui Song
// Copyright (C) 2018 Jinhui Song
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#include "WayPointMobility.h"
#include <omnetpp.h>

Define_Module(WayPointMobility);


double WayPointMobility::arc2deg(double angle, Coord a, Coord b)
{
    angle *= 180 / M_PI;
    if((b.y - a.y) * angle < 0 || (angle == 0 && b.x < a.x))
        angle += 180;             // make angle located in [0, 360]
    else if(angle < 0) angle += 360;
    return angle;
}

Coord WayPointMobility::Coord2cos(Coord begin, Coord end)
{
    Coord coses;
    coses.x = (end.x - begin.x) / end.distance(begin);
    coses.y = (end.y - begin.y) / end.distance(begin);
    coses.z = (end.z - begin.z) / end.distance(begin);
    return coses;
}

void WayPointMobility::initialize(int stage)
{
    BaseMobility::initialize(stage);
    EV << "initializing Circle Mobility stage " << stage << endl;
    if(stage == 0)
    {
        move.setSpeed(par("speed").doubleValue());
        point.push_back(Coord(par("X1").doubleValue(), par("Y1").doubleValue(), par("Z1").doubleValue()));
        if(par("X2").doubleValue() != -1) 
            point.push_back(Coord(par("X2").doubleValue(), par("Y2").doubleValue(), par("Z2").doubleValue()));
        if(par("X3").doubleValue() != -1) 
            point.push_back(Coord(par("X3").doubleValue(), par("Y3").doubleValue(), par("Z3").doubleValue()));
    }
    else if(stage == 1)
    {
        stepTarget = move.getStartPos();
        coses = (point.at(0) - stepTarget) / stepTarget.distance(point.at(0));
        lineNo = 0;
    }
}

void WayPointMobility::fixIfHostGetsOutside()
{
    Coord dummy = Coord::ZERO;
    double angle = arc2deg( atan(coses.y / coses.x), point.at(lineNo), stepTarget );
    handleIfOutside(WRAP, stepTarget, dummy, dummy, angle);
}

void WayPointMobility::makeMove()
{
    debugEV << "start makeMove " << move.info() << endl;
    move.setStart(stepTarget, simTime());
    
    double r = move.getSpeed() * updateInterval.dbl();
    stepTarget += coses * r;
    int nextNo = (lineNo + 1) % point.size();
    Coord temp = stepTarget - point.at(lineNo);
    if(temp.x * coses.x >= 0 && temp.y * coses.y >= 0 && temp.z * coses.z >= 0)    // step is in front of point
    {
        stepTarget = point.at(lineNo);       // if current step target reaches the next point
        lineNo = nextNo;                     // fly through a polygon but not come and go through same path
        coses = (point.at(nextNo) - stepTarget) / point.at(nextNo).distance(stepTarget);
    }
    
    move.setDirectionByTarget(stepTarget);
    EV << "new step target of circle: " << stepTarget.info() << endl;
    fixIfHostGetsOutside();
    
}














