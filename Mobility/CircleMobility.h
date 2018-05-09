/* -*- mode:c++ -*- ********************************************************
 * file:        CircleMobility.h
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
 **************************************************************************/

#ifndef CIRCLE_MOBILITY_H
#define CIRCLE_MOBILITY_H

#include "veins/base/modules/BaseMobility.h"

// developed from linear mobility, the example

class MIXIM_API CircleMobility : public BaseMobility
{
protected:
    double angle; 
    double R;
    Coord center;       // input: only use x & y to define a 2-D circle
    Coord stepTarget;
    
public:
    virtual void initialize(int);
    virtual double arc2deg(double, Coord, Coord );

protected:
    virtual void makeMove();
    virtual void fixIfHostGetsOutside();
    
};

#endif
