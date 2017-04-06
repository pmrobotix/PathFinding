/*
 * pmr_tools.h
 *
 *  Created on: Apr 6, 2017
 *      Author: gmo
 */

#ifndef PMR_TOOLS_H_
#define PMR_TOOLS_H_


/*****************************************************************************/
/* General purpose features                                                  */
/*****************************************************************************/

/* Tools */

#define TOOLS_EPSILON 0.001


int tools_quasi_equal(float a, float b);


int tools_is_between(float a, float b, float x);


float tools_distance(float x1, float y1, float x2, float y2);


#endif /* PMR_TOOLS_H_ */
