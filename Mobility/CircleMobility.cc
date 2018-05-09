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

#include "CircleMobility.h"
#include <omnetpp.h>

Define_Module(CircleMobility);

double CircleMobility::arc2deg(double angle, Coord a, Coord b)
{
    angle *= 180 / M_PI;
    if((b.y - a.y) * angle < 0 || (angle == 0 && b.x < a.x))
        angle += 180;             // make angle located in [0, 360]
    else if(angle < 0) angle += 360;
    return angle;
}

void CircleMobility::initialize(int stage)
{
    BaseMobility::initialize(stage);
    EV << "initializing Circle Mobility stage " << stage << endl;
    if(stage == 0)
    {
        move.setSpeed(par("speed").doubleValue());
        center.x = par("X1").doubleValue();
        center.y = par("Y1").doubleValue();
    }
    else if(stage == 1)
    {
        stepTarget = move.getStartPos();

        // calculate angle and R from center and stepTarget here, test passed
        center.z = stepTarget.z;
        R = center.distance(stepTarget);
        angle = arc2deg( atan((stepTarget.y - center.y) / (stepTarget.x - center.x)), center, stepTarget );
    }
}

void CircleMobility::fixIfHostGetsOutside()
{
    Coord dummy = Coord::ZERO;
    handleIfOutside(WRAP, stepTarget, dummy, dummy, angle);
}

void CircleMobility::makeMove()
{
    debugEV << "start makeMove " << move.info() <<endl;
    move.setStart(stepTarget, simTime());
    
    double r = move.getSpeed() * updateInterval.dbl();
    angle += 2 * asin(r / 2 / R) * 180 / M_PI;
    stepTarget.x += r * cos(M_PI * angle / 180);
    stepTarget.y += r * sin(M_PI * angle / 180);
   
    move.setDirectionByTarget(stepTarget);
    
    EV << "new step target of circle: " << stepTarget.info() << endl;
    fixIfHostGetsOutside();
}




