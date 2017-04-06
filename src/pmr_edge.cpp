#include <cstddef>
#include "pmr_edge.h"
#include "pmr_tools.h"
#include <math.h>
/* Edge **********************************************************************/


Edge* edge_new(Node* node1, Node* node2)
{
    Edge* self = new Edge;

    self->node1 = node1;
    self->node2 = node2;
    self->a = INFINITY;
    self->b = INFINITY;
    self->enabled = 1;
    self->length = 0.0;
    self->zone_internal = 0;

    return self;
}


void edge_free(Edge* self)
{
    delete self;
}


int edge_links(Edge* self, Node* node1, Node* node2)
{
    return (self->node1 == node1 && self->node2 == node2) || (self->node1 == node2 && self->node2 == node1);
}


Node* edge_other_node(Edge* self, Node* node)
{
    if (self->node1 == node) {
        return self->node2;
    } else if (self->node2 == node) {
        return self-> node1;
    }
    return NULL;
}

void edge_update(Edge* self)
{
    if (tools_quasi_equal(self->node1->x, self->node2->x)) {
        /* Vertical edge */
        self->a = INFINITY;
        self->b = INFINITY;
    } else {
        /* General case */
        self->a = (self->node2->y - self->node1->y) / (self->node2->x - self->node1->x);
        self->b = self->node1->y - self->a * self->node1->x;
    }
    self->length = tools_distance(self->node1->x, self->node1->y, self->node2->x, self->node2->y);
}


int edge_contains(Edge* self, float x, float y)
{
    int ok = 0;
    if (!isfinite(self->a)) {
        /* Vertical edge */
        ok = tools_quasi_equal(x, self->node1->x) &&
             tools_is_between(self->node1->y, self->node2->y, y);
    } else if (tools_quasi_equal(self->a, 0.0)) {
        /* Horizontal edge */
        ok = tools_quasi_equal(self->b, y) &&
            tools_is_between(self->node1->x, self->node2->x, x);
    } else {
        /* General case */
        /* Check that the point is on the line */
        ok = tools_quasi_equal(self->a * x + self->b, y);
        /* Check that the point is in the segment bounds */
        ok &= tools_is_between(self->node1->x, self->node2->x, x);
        ok &= tools_is_between(self->node1->y, self->node2->y, y);
    }
    return ok;
}


int edge_intersects(Edge* self, Edge* other)
{
    float cross_x = 0.0;
    float cross_y = 0.0;

    if ( self->node1 == other->node1 || self->node1 == other->node2 || self->node2 == other->node1 || self->node2 == other->node2) {
        return 0;
    }
    if (node_coords_equal(self->node1, other->node1) ||
        node_coords_equal(self->node1, other->node2) ||
        node_coords_equal(self->node2, other->node1) ||
        node_coords_equal(self->node2, other->node2)) {
        return 0;
    }
    if (!isfinite(self->a) && !isfinite(other->a)) {
        /* Two vertical lines */
        return tools_quasi_equal(self->node1->x, other->node1->x) &&
                (edge_contains(self, other->node1->x, other->node1->y) ||
                 edge_contains(self, other->node2->x, other->node2->y));
    }
    if (!isfinite(self->a)) {
        cross_x = self->node1->x;
        cross_y = other->a * cross_x + other->b;
    } else if (!isfinite(other->a)) {
        cross_x = other->node1->x;
        cross_y = self->a * cross_x + self->b;
    } else if (!tools_quasi_equal(self->a, other->a)) {
        cross_x = (other->b - self->b) / (self->a - other->a);
        cross_y = self->a * cross_x + self->b;
    } else {
        /* Two segments on the same line */
        return tools_quasi_equal(self->b, other->b) &&
                (tools_is_between(self->node1->x, self->node2->x, other->node1->x) ||
                 tools_is_between(self->node1->x, self->node2->x, other->node2->x));
    }

    return edge_contains(self, cross_x, cross_y) && edge_contains(other, cross_x, cross_y);
}
