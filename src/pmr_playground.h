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

typedef unsigned int PlaygroundObjectID;

class Playground {
private:
    class PlaygroundImpl;
    std::unique_ptr<PlaygroundImpl> playground_impl;
public:
    const PlaygroundObjectID INVALID = UINT_MAX;
    PlaygroundObjectID this_robot = INVALID;
    PlaygroundObjectID teammate = INVALID;
    PlaygroundObjectID opponent_1 = INVALID;
    PlaygroundObjectID opponent_2 = INVALID;
    Playground();
    virtual ~Playground();

    virtual Playground* add_circle(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_quarter_circle(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_quarter_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_half_circle(float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_half_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments);
    virtual Playground* add_segment(float x1, float y1, float x2, float y2, float radius);
    virtual Playground* add_segment(PlaygroundObjectID& id, float x1, float y1, float x2, float y2, float radius);
    virtual Playground* add_rectangle(float x, float y, float dx, float dy, float angle);
    virtual Playground* add_rectangle(PlaygroundObjectID& id, float x, float y, float dx, float dy, float angle);
    virtual Playground* add_convex_body(float x, float y, const std::vector<Point*>& relative_points, float angle);
    virtual Playground* add_convex_body(PlaygroundObjectID& id, float x, float y, const std::vector<Point*>& relative_points, float angle);

    virtual Playground* move(PlaygroundObjectID id, float dx, float dy);
    virtual Playground* rotate(PlaygroundObjectID id, float angle);
    virtual Playground* change_shape(PlaygroundObjectID id, const std::vector<Point*>& points);
    virtual Playground* get_shape(std::vector<Point*>& points, PlaygroundObjectID id);

    virtual Playground* enable(PlaygroundObjectID id, bool enabled);
    virtual Playground* compute_edges();
    virtual Playground* evaluate_path(const Point& start, const Point& end);
};

#endif /* PMR_PLAYGROUND_H_ */
