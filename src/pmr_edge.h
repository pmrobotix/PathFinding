/*
 * pmr_edge.h
 *
 *  Created on: Apr 6, 2017
 *      Author: gmo
 */

#ifndef PMR_EDGE_H_
#define PMR_EDGE_H_

#include "pmr_node.h"

/* Edge **********************************************************************/

struct Edge
{
    Node* node1;
    Node* node2;
    float a;
    float b;
    int enabled;
    float length;
    int zone_internal;
};


Edge* edge_new(Node* node1, Node* node2);


void edge_free(Edge* self);


int edge_links(Edge* self, Node* node1, Node* node2);


Node* edge_other_node(Edge* self, Node* node);

void edge_update(Edge* self);


int edge_contains(Edge* self, float x, float y);


int edge_intersects(Edge* self, Edge* other);


#endif /* PMR_EDGE_H_ */
