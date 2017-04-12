/*
 * pmr_zone.h
 *
 *  Created on: Apr 6, 2017
 *      Author: gmo
 */

#ifndef PMR_ZONE_H_
#define PMR_ZONE_H_

#include <vector>
#include "pmr_point.h"
#include "pmr_edge.h"
/* Zone **********************************************************************/


struct Zone
{
    Node** nodes;
    Edge** edges;
    unsigned int nodes_count;
    unsigned int id;
    int enabled;
    bool is_enabled;
    bool is_detected;
    float dx;
    float dy;
};


Zone* zone_new(int id, std::vector<Point>& points_list);


void zone_free(Zone* self);


void zone_update(Zone* self, const std::vector<Point>& points_list);


int zone_is_internal_edge(Zone* self, Edge* edge);


int zone_contains_node(Zone* self, Node* node);


void zone_get_center(Zone* self, float* x, float* y);


bool zone_is_enabled(Zone * self);


bool zone_is_detected(Zone * self);


void zone_set_is_enabled(Zone * self, bool is_enabled);


void zone_set_is_detected(Zone * self, bool is_detected);


#endif /* PMR_ZONE_H_ */
