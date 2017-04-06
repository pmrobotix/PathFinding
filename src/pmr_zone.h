/*
 * pmr_zone.h
 *
 *  Created on: Apr 6, 2017
 *      Author: gmo
 */

#ifndef PMR_ZONE_H_
#define PMR_ZONE_H_

#include <vector>
#include "pmr_edge.h"
/* Zone **********************************************************************/


struct Zone
{
    Node** nodes;
    Edge** edges;
    unsigned int nodes_count;
    int id;
    int enabled;
    float dx;
    float dy;
};


Zone* zone_new(int id, std::vector<Node>& points_list);


void zone_free(Zone* self);


void zone_update(Zone* self, std::vector<Node>& points_list);


int zone_is_internal_edge(Zone* self, Edge* edge);


int zone_contains_node(Zone* self, Node* node);

void zone_get_center(Zone* self, float* x, float* y);



#endif /* PMR_ZONE_H_ */
