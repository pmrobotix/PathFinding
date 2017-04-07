/*
 * pmr_node.h
 *
 *  Created on: Apr 5, 2017
 *      Author: gmo
 */

#ifndef PMR_POINT_H_
#define PMR_POINT_H_

/* Point **********************************************************************/

struct Point
{
    float x;
    float y;
};

Point* point_new(float x, float y);


void point_free(Point* self);


#endif /* PMR_POINT_H_ */
