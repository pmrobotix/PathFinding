/*
 * pmr_zone.cpp
 *
 *  Created on: Apr 6, 2017
 *      Author: gmo
 */
/* Zone **********************************************************************/

#include <cstddef>
#include "pmr_zone.h"


/* Exceptions */
class RuntimeErrorBadCountInZoneUpdate: public std::exception
{
    virtual const char* what() const throw() {
        return "Unable to update the zone. points count don't match.";
    }
} badCount;


Zone* zone_new(int id, std::vector<Point>& points_list)
{
    std::vector<Point>::iterator it;
    Zone* self = new Zone;
    std::size_t size = points_list.size();
    self->nodes = new Node*[size];
    self->edges = new Edge*[size];
    self->nodes_count = 0;
    self->id = id;
    self->enabled = 1;
    self->dx = 0.0;
    self->dy = 0.0;

    for (it = points_list.begin(); it < points_list.end(); it++) {
        self->nodes[self->nodes_count++] = node_new(it->x, it->y);
    }

    return self;
}


void zone_free(Zone* self)
{
    delete self->edges;
    delete self->nodes;
    delete self;
}


void zone_update(Zone* self, std::vector<Point>& points_list)
{
    unsigned int i = 0;
    unsigned int points_count = (unsigned int) points_list.size();
    std::vector<Point>::iterator it;

    if (points_count != self->nodes_count) {
        throw badCount;
    }

    for (it = points_list.begin(); it < points_list.end(); it++,i++) {
        Node* node = self->nodes[i];
        node->x = it->x;
        node->y = it->y;
    }
}


int zone_is_internal_edge(Zone* self, Edge* edge)
{
    size_t i;
    int node1InZone = 0;
    int node2InZone = 0;
    Node* previous_node = self->nodes[self->nodes_count - 1];

    for (i = 0; i < self->nodes_count; ++i) {
        Node* node = self->nodes[i];
        if (edge_links(edge, previous_node, node)) {
            return 0;
        }
        node1InZone |= edge->node1 == node;
        node2InZone |= edge->node2 == node;
        if (node1InZone && node2InZone) {
            return 1;
        }

        previous_node = node;
    }

    return 0;
}


int zone_contains_node(Zone* self, Node* node)
{
    /* WARNING ! This function works only for convex polygon zones */

    Node* node1 = NULL;
    Node* node2 = NULL;
    unsigned int i = 0;
    int sign = 0;
    int k = 0;
    float dx1 = 0.0;
    float dy1 = 0.0;
    float dx2 = 0.0;
    float dy2 = 0.0;
    float cross_product = 0.0;

    node1 = self->nodes[self->nodes_count - 1];
    for (i = 0; i < self->nodes_count; ++i) {
        node2 = self->nodes[i];
        if (node1 == node) {
            return 0;
        }
        dx1 = node2->x - node1->x;
        dy1 = node2->y - node1->y;
        dx2 = node->x - node1->x;
        dy2 = node->y - node1->y;
        cross_product = dx1 * dy2 - dy1 * dx2;
        k = cross_product >= 0.0 ? 1 : -1;
        if (sign == 0) {
            sign = k;
        } else if (sign != k) {
            return 0;
        }
        node1 = node2;
    }
    return 1;
}


void zone_get_center(Zone* self, float* x, float* y)
{
    unsigned int i = 0;

    *x = 0.0;
    *y = 0.0;
    for (i = 0; i < self->nodes_count; ++i) {
        Node* node = self->nodes[i];
        *x += node->x;
        *y += node->y;
    }
    *x /= self->nodes_count;
    *y /= self->nodes_count;
}
