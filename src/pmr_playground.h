/*
 * pmr_playground.h
 *
 *  Created on: Apr 8, 2017
 *      Author: gmo
 */

#ifndef PMR_PLAYGROUND_H_
#define PMR_PLAYGROUND_H_

#include <climits>
#include <memory>
#include <vector>
#include "pmr_point.h"
#include "pmr_path_result.h"
#include "pmr_pathfinding.h"

typedef unsigned int PlaygroundObjectID;

class Playground {
private:
    class PlaygroundImpl;
    std::unique_ptr<PlaygroundImpl> playground_impl;
public:
    static const PlaygroundObjectID INVALID = UINT_MAX;

    Playground(float field_x1, float field_y1, float field_x2, float field_y2,
            float zone_escape_increment, float zone_escape_max_increment);
    virtual ~Playground();

    virtual Playground* add_circle(float x, float y, float radius, unsigned int num_segments);
    virtual Playground* add_circle(PlaygroundObjectID& id, float x, float y, float radius, unsigned int num_segments);
    virtual Playground* add_quarter_circle(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_quarter_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_half_circle(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_half_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_segment(float x1, float y1, float x2, float y2, float radius);
    virtual Playground* add_segment(PlaygroundObjectID& id, float x1, float y1, float x2, float y2, float radius);
    virtual Playground* add_rectangle(float x, float y, float dx, float dy, float angle);
    virtual Playground* add_rectangle(PlaygroundObjectID& id, float x, float y, float dx, float dy, float angle);
    virtual Playground* add_rectangle_lower_left(float x, float y, float dx, float dy, float angle);
    virtual Playground* add_rectangle_lower_left(PlaygroundObjectID& id, float x, float y, float dx, float dy, float angle);
    virtual Playground* add_convex_body(float x, float y, const std::vector<Point*>& relative_points, float angle);
    virtual Playground* add_convex_body(PlaygroundObjectID& id, float x, float y, const std::vector<Point*>& relative_points, float angle);

    virtual Playground* move(PlaygroundObjectID id, float dx, float dy);
    virtual Playground* change_shape(PlaygroundObjectID id, const std::vector<Point>& points);
    virtual Playground* get_shape(std::vector<Point> * & points, PlaygroundObjectID id);

    virtual Playground* enable(PlaygroundObjectID id, bool is_enabled);
    virtual Playground* detect(PlaygroundObjectID id, bool is_detected);
    virtual Playground* compute_edges();
    virtual Playground* synchronize();
    virtual Playground* find_path(FoundPath * & path, Point& start, Point& end);

    virtual unsigned int get_nodes_count();
    virtual unsigned int get_nodes_count(PlaygroundObjectID id);
    virtual unsigned int get_edges_count();
    virtual unsigned int get_zones_count();

    virtual PathFinder* get_path_finder();
};

#endif /* PMR_PLAYGROUND_H_ */
