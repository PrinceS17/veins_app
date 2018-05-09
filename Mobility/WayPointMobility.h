/* -*- mode:c++ -*- ********************************************************
 * file:        LinearMobility.h
 *
 * author:      Jinhui Song
 *
 * Copyright    (C) 2018 Jinhui Song
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 **************************************************************************/

#ifndef WAYPOINTMOBILITY_H
#define WAYPOINTMOBILITY_H

#include "veins/base/modules/BaseMobility.h"
using namespace std;

/**
 * This mobility module specifies the point of a line, 
 * which is different from default linear mobility. 
 * 
 * Note: want it to support 3D movement. 
 * @author Jinhui Song
 */
class MIXIM_API WayPointMobility : public BaseMobility
{
protected:
    vector<Coord> point;       // input parameter, decide current state parameters below
    int lineNo;                // current line: point[lineNo - 1] -> point[lineNo], lineNo is destination of current line
    
    Coord stepTarget;
    Coord coses;
    
public:
    virtual void initialize(int);
    virtual double arc2deg(double, Coord, Coord);
    virtual Coord Coord2cos(Coord, Coord);     // use Coord of begin and end to calculate 3 cos values
protected:
    virtual void makeMove();
    virtual void fixIfHostGetsOutside();
};

#endif
