/*
 * pmr_symetrical_pg.cpp
 *
 *  Created on: 11 mai 2019
 *      Author: pmrobotix
 */

#include "pmr_symmetrical_pg.h"
#include <cmath>

class SymmetricalPlayground::SymmetricalPlaygroundImpl {
public:
    float symmetry_ax;

    inline float symmetryX(float x) {
        return 2.0f * symmetry_ax - x;
    }
    inline float relativeSymmetryX(float x) {
        return -x;
    }
    inline float relativeSymmetryAngle(float angle, float innerAngle) {
        return M_PI-innerAngle-angle;
    }
    void compute_sym_convex_body(float x, float y,
        const std::vector<Point*>& relative_points, float angle,
        Point absolute_points[]) {
        float cos_a = cos(angle);
        float sin_a = sin(angle);
        unsigned int i = 0;

        std::vector<Point*>::const_iterator points_it;
        for (points_it = relative_points.begin(); points_it < relative_points.end(); points_it++, i++) {
            const Point * relative_point = *points_it;
            absolute_points[i].x = symmetryX(x + relative_point->x * cos_a - relative_point->y * sin_a);
            absolute_points[i].y = y + relative_point->x * sin_a + relative_point->y * cos_a;
        }
    }
};

SymmetricalPlayground::SymmetricalPlayground(float field_x1, float field_y1, float field_x2, float field_y2,
        float zone_escape_increment, float zone_escape_max_increment, float symmetrical_ax):
                Playground::Playground(field_x1, field_y1,
                field_x2, field_y2, zone_escape_increment, zone_escape_max_increment),
                sym_playground_impl { new SymmetricalPlaygroundImpl }
{
    sym_playground_impl->symmetry_ax = symmetrical_ax;
}

SymmetricalPlayground::~SymmetricalPlayground()
{
    // destruction is done automatically
}

SymmetricalPlayground* SymmetricalPlayground::add_circle_symmetrical(float x, float y,
        float radius, unsigned int num_segments) {
    PlaygroundObjectID id = INVALID;
    PlaygroundObjectID id_sym = INVALID;
    add_circle_symmetrical(id, id_sym, x, y, radius, num_segments);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_circle_symmetrical(
        PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float radius,
        unsigned int num_segments) {
    add_circle(id, x, y, radius, num_segments);
    add_circle(id_sym, sym_playground_impl->symmetryX(x), y, radius, num_segments);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_quarter_circle_symmetrical(float x,
        float y, float radius, float angle, unsigned int num_segments) {
    PlaygroundObjectID id = INVALID;
    PlaygroundObjectID id_sym = INVALID;
    add_quarter_circle_symmetrical(id, id_sym, x, y, radius, angle, num_segments);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_quarter_circle_symmetrical(
        PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float radius, float angle,
        unsigned int num_segments) {
    add_quarter_circle(id, x, y, radius, angle, num_segments);
    add_quarter_circle(id_sym, sym_playground_impl->symmetryX(x), y, radius,
            sym_playground_impl->relativeSymmetryAngle(angle, M_PI/2.0f), num_segments);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_half_circle_symmetrical(float x, float y,
        float radius, float angle, unsigned int num_segments) {
    PlaygroundObjectID id = INVALID;
    PlaygroundObjectID id_sym = INVALID;
    add_half_circle_symmetrical(id, id_sym, x, y, radius, angle, num_segments);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_half_circle_symmetrical(
        PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float radius, float angle,
        unsigned int num_segments) {
    add_half_circle(id, x, y, radius, angle, num_segments);
    add_half_circle(id_sym, sym_playground_impl->symmetryX(x), y, radius,
            sym_playground_impl->relativeSymmetryAngle(angle, M_PI), num_segments);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_segment_symmetrical(float x1, float y1,
        float x2, float y2, float radius) {
    PlaygroundObjectID id = INVALID;
    PlaygroundObjectID id_sym = INVALID;
    add_segment_symmetrical(id, id_sym, x1, y1, x2, y2, radius);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_segment_symmetrical(
        PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x1, float y1, float x2, float y2,
        float radius) {
    add_segment(id, x1, y1, x2, y2, radius);
    add_segment(id_sym,
            sym_playground_impl->symmetryX(x2), y2,
            sym_playground_impl->symmetryX(x1), y1,
            radius);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_rectangle_symmetrical(float x, float y,
        float dx, float dy, float angle) {
    PlaygroundObjectID id = INVALID;
    PlaygroundObjectID id_sym = INVALID;
    add_rectangle_symmetrical(id, id_sym, dx, dy, angle);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_rectangle_symmetrical(
        PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float dx, float dy,
        float angle) {
    add_rectangle(id, x, y, dx, dy, angle);
    add_rectangle(id_sym,
            sym_playground_impl->symmetryX(x),
            y,
            dx, dy,
            sym_playground_impl->relativeSymmetryX(angle));
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_convex_body_symmetrical(float x, float y,
        const std::vector<Point*>& relative_points, float angle) {
    PlaygroundObjectID id = INVALID;
    PlaygroundObjectID id_sym = INVALID;
    add_convex_body_symmetrical(id, id_sym, x, y, relative_points, angle);
    return this;
}

SymmetricalPlayground* SymmetricalPlayground::add_convex_body_symmetrical(
        PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y,
        const std::vector<Point*>& relative_points, float angle) {
    unsigned int num_points = relative_points.size();
    Point * abs_points_pointer = new Point[num_points];
    std::vector<Point*> absolute_point_refs = std::vector<Point*>(num_points);

    add_convex_body(id, x, y, relative_points, angle);

    /* Easy, but not elegant method to compute the symmetries
     * It duplicates the code from pmr_playground.cpp
     */
    sym_playground_impl->compute_sym_convex_body(x, y, relative_points, angle, abs_points_pointer);
    std::vector<Point>::const_iterator points_it;
    for (unsigned int i = 0; i<num_points; i++) {
        absolute_point_refs[i] = abs_points_pointer + i;
    }
    add_convex_body(id_sym,
            0.0f,
            0.0f,
            absolute_point_refs,
            0.0f);
    delete[] abs_points_pointer;
    return this;
}
