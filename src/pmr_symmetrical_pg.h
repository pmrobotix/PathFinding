/*
 * pmr_symetrical_pg.h
 *
 *  Created on: 11 mai 2019
 *      Author: pmrobotix
 */

#ifndef PMR_SYMMETRICAL_PG_H_
#define PMR_SYMMETRICAL_PG_H_

#include "pmr_playground.h"

class SymmetricalPlayground: public Playground {
private:
    class SymmetricalPlaygroundImpl;
    std::unique_ptr<SymmetricalPlaygroundImpl> sym_playground_impl;
public:
    SymmetricalPlayground(float field_x1, float field_y1, float field_x2, float field_y2,
            float zone_escape_increment, float zone_escape_max_increment, float symmetrical_ax);
    virtual ~SymmetricalPlayground();

    virtual SymmetricalPlayground* add_circle_symmetrical(float x, float y, float radius, unsigned int num_segments);
    virtual SymmetricalPlayground* add_circle_symmetrical(PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float radius, unsigned int num_segments);
    virtual SymmetricalPlayground* add_quarter_circle_symmetrical(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual SymmetricalPlayground* add_quarter_circle_symmetrical(PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual SymmetricalPlayground* add_half_circle_symmetrical(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual SymmetricalPlayground* add_half_circle_symmetrical(PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual SymmetricalPlayground* add_segment_symmetrical(float x1, float y1, float x2, float y2, float radius);
    virtual SymmetricalPlayground* add_segment_symmetrical(PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x1, float y1, float x2, float y2, float radius);
    virtual SymmetricalPlayground* add_rectangle_symmetrical(float x, float y, float dx, float dy, float angle);
    virtual SymmetricalPlayground* add_rectangle_symmetrical(PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, float dx, float dy, float angle);
    virtual SymmetricalPlayground* add_convex_body_symmetrical(float x, float y, const std::vector<Point*>& relative_points, float angle);
    virtual SymmetricalPlayground* add_convex_body_symmetrical(PlaygroundObjectID& id, PlaygroundObjectID& id_sym, float x, float y, const std::vector<Point*>& relative_points, float angle);
};

#endif /* PMR_SYMMETRICAL_PG_H_ */

