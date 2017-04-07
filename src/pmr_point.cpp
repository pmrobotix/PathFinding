/*
 * pmr_point.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: gmo
 */

#include "pmr_point.h"

Point* point_new(float x, float y)
{
    Point* self = new Point;
    self->x = x;
    self->y = y;
    return self;
}


void point_free(Point* self)
{
    delete self;
}
