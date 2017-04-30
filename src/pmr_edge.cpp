#include <cstddef>
#include "pmr_edge.h"
#include "pmr_tools.h"
#include <cmath>
#include "pmr_debug_logging.h"
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
    PMR_DBG("Node 1: " << self->node1->x << ", " << self->node1->y << " Node 2: " << self->node2->x << ", " << self->node2->y << " a: " << self->a << " b: " << self->b << " length: " << self->length);
}


int edge_contains(Edge* self, float x, float y)
{
    int ok = 0;
    if (!std::isfinite(self->a)) {
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


// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
static bool onSegment(Node & p, Node & q, Node & r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
       return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
static int orientation(Node & p, Node & q, Node & r)
{
    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
static bool doIntersect(Node & p1, Node & q1, Node & p2, Node & q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}


int edge_intersects(Edge* self, Edge* other)
{
//    float cross_x = 0.0;
//    float cross_y = 0.0;
    bool result;

    if ( self->node1 == other->node1 || self->node1 == other->node2 || self->node2 == other->node1 || self->node2 == other->node2) {
        return 0;
    }
//    if (node_coords_equal(self->node1, other->node1) ||
//        node_coords_equal(self->node2, other->node2) ||
//        node_coords_equal(self->node2, other->node1) ||
//        node_coords_equal(self->node1, other->node2)) {
//        return 0;
//    }

    result = doIntersect(*(self->node1), *(self->node2), *(other->node1), *(other->node2));
    return result ? 1 : 0;
//    if (!std::isfinite(self->a) && !std::isfinite(other->a)) {
//        /* Two vertical lines */
//        return tools_quasi_equal(self->node1->x, other->node1->x) &&
//                (edge_contains(self, other->node1->x, other->node1->y) ||
//                 edge_contains(self, other->node2->x, other->node2->y));
//    }
//    if (!std::isfinite(self->a)) {
//        cross_x = self->node1->x;
//        cross_y = other->a * cross_x + other->b;
//    } else if (!std::isfinite(other->a)) {
//        cross_x = other->node1->x;
//        cross_y = self->a * cross_x + self->b;
//    } else if (!tools_quasi_equal(self->a, other->a)) {
//        cross_x = (other->b - self->b) / (self->a - other->a);
//        cross_y = self->a * cross_x + self->b;
//    } else {
//        /* Two segments on the same line */
//        return tools_quasi_equal(self->b, other->b) &&
//                (tools_is_between(self->node1->x, self->node2->x, other->node1->x) ||
//                 tools_is_between(self->node1->x, self->node2->x, other->node2->x));
//    }
//
//    return edge_contains(self, cross_x, cross_y) && edge_contains(other, cross_x, cross_y);
}
