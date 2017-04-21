/*
 * pmr_playground.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: gmo
 */

#include "pmr_playground.h"
#include "pmr_pathfinding.h"
#include <cmath>

class Playground::PlaygroundImpl {
public:
    static constexpr float PI = 3.1415927;
    PathFinder * my_playground;
    void enable(PlaygroundObjectID teammate, PlaygroundObjectID opponent_1, PlaygroundObjectID opponent_2, int enabled);
};

Playground::Playground(float field_x1, float field_y1, float field_x2, float field_y2,
        float zone_escape_increment, float zone_escape_max_increment) : playground_impl { new PlaygroundImpl }
{
    playground_impl->my_playground = new PathFinder;
    pathfinder_init(playground_impl->my_playground, field_x1, field_y1, field_x2, field_y2, zone_escape_increment, zone_escape_max_increment);
}

Playground::~Playground() {
    pathfinder_dealloc(playground_impl->my_playground);
}


Playground* Playground::add_circle(float x, float y, float radius, unsigned int num_segments)
{
    PlaygroundObjectID id = INVALID;
    return add_circle(id, x, y, radius, num_segments);
}


Playground* Playground::add_circle(PlaygroundObjectID& id, float x, float y, float radius, unsigned int num_segments)
{
    std::vector<Point> points_list = std::vector<Point>(num_segments);
    const float angle_increment = 2.0 * playground_impl->PI / ((float) num_segments);
    for (unsigned int i = 0; i<num_segments; i++) {
        float angle = ((float) i) * angle_increment;
        Point p;
        p.x = x + radius * cos(angle);
        p.y = y + radius * sin(angle);
        points_list[i] = p;
    }
    id = pathfinder_add_zone(playground_impl->my_playground, points_list);
    return this;
}


Playground* Playground::add_quarter_circle(float x, float y, float radius, float angle, unsigned int num_segments)
{
    PlaygroundObjectID id = INVALID;
    return add_quarter_circle(id, x, y, radius, angle, num_segments);
}


Playground* Playground::add_quarter_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments)
{
    std::vector<Point> points_list = std::vector<Point>(num_segments);
    const float angle_increment = 0.5 * playground_impl->PI / (((float) num_segments)-1);
    for (unsigned int i = 0; i<num_segments; i++) {
        Point p;
        p.x = x + radius * cos(angle);
        p.y = y + radius * sin(angle);
        points_list[i] = p;
        angle += angle_increment;
    }
    id = pathfinder_add_zone(playground_impl->my_playground, points_list);
    return this;
}


Playground* Playground::add_half_circle(float x, float y, float radius, float angle, unsigned int num_segments)
{
    PlaygroundObjectID id = INVALID;
    return add_half_circle(id, x, y, radius, angle, num_segments);;
}


Playground* Playground::add_half_circle(PlaygroundObjectID& id, float x, float y, float radius, float angle, unsigned int num_segments)
{
    std::vector<Point> points_list = std::vector<Point>(num_segments);
    const float angle_increment = 1.0 * playground_impl->PI / (((float) num_segments)-1);
    for (unsigned int i = 0; i<num_segments; i++) {
        Point p;
        p.x = x + radius * cos(angle);
        p.y = y + radius * sin(angle);
        points_list[i] = p;
        angle += angle_increment;
    }
    id = pathfinder_add_zone(playground_impl->my_playground, points_list);
    return this;
}


Playground* Playground::add_segment(float x1, float y1, float x2, float y2, float radius)
{
    PlaygroundObjectID id = INVALID;
    return add_segment(id, x1, y1, x2, y2, radius);
}


Playground* Playground::add_segment(PlaygroundObjectID& id, float x1, float y1, float x2, float y2, float radius)
{
    unsigned int num_segments = 4;
    std::vector<Point> points_list = std::vector<Point>(num_segments);
    float angle = atan2(y2-y1, x2-x1) + 0.5 * playground_impl->PI;
    const float angle_increment = playground_impl->PI / (((float) num_segments)-1);
    for (unsigned int i = 0; i<num_segments; i++) {
        Point p;
        p.x = x1 + radius * cos(angle);
        p.y = y1 + radius * sin(angle);
        points_list[i] = p;
        angle += angle_increment;
    }
    angle = atan2(y2-y1, x2-x1) - 0.5 * playground_impl->PI;
    for (unsigned int i = 0; i<num_segments; i++) {
        Point p;
        p.x = x2 + radius * cos(angle);
        p.y = y2 + radius * sin(angle);
        points_list.push_back(p);
        angle += angle_increment;
    }
    id = pathfinder_add_zone(playground_impl->my_playground, points_list);
    return this;
}


Playground* Playground::add_rectangle(float x, float y, float dx, float dy, float angle)
{
    PlaygroundObjectID id = INVALID;
    return add_rectangle(id, x, y, dx, dy, angle);
}


Playground* Playground::add_rectangle(PlaygroundObjectID& id, float x, float y, float dx, float dy, float angle)
{
    float width_2 = dx / 2.0;
    float height_2 = dy / 2.0;
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    unsigned int i = 0;
    std::vector<Point> points_list = std::vector<Point>(4);
    Point p;
    p.x = x - width_2 * sin_a + height_2 * cos_a;
    p.y = y + width_2 * cos_a + height_2 * sin_a;
    points_list[i++] = p;
    p.x = x + width_2 * sin_a + height_2 * cos_a;
    p.y = y - width_2 * cos_a + height_2 * sin_a;
    points_list[i++] = p;
    p.x = x + width_2 * sin_a - height_2 * cos_a;
    p.y = y - width_2 * cos_a - height_2 * sin_a;
    points_list[i++] = p;
    p.x = x - width_2 * sin_a - height_2 * cos_a;
    p.y = y + width_2 * cos_a - height_2 * sin_a;
    points_list[i++] = p;
    id = pathfinder_add_zone(playground_impl->my_playground, points_list);
    return this;
}


Playground* Playground::add_convex_body(float x, float y, const std::vector<Point*>& relative_points, float angle)
{
    PlaygroundObjectID id = INVALID;
    return add_convex_body(id, x, y, relative_points, angle);
}


Playground* Playground::add_convex_body(PlaygroundObjectID& id, float x, float y, const std::vector<Point*>& relative_points, float angle)
{

    float cos_a = cos(angle);
    float sin_a = sin(angle);
    unsigned int i = 0;
    std::vector<Point> points_list = std::vector<Point>(relative_points.size());
    Point p;
    std::vector<Point*>::const_iterator points_it;
    for (points_it = relative_points.begin(); points_it < relative_points.end(); points_it++) {
        const Point * relative_point = *points_it;
        p.x = x + relative_point->x * cos_a - relative_point->y * sin_a;
        p.y = y + relative_point->x * sin_a + relative_point->y * cos_a;
        points_list[i++] = p;
    }
    id = pathfinder_add_zone(playground_impl->my_playground, points_list);
    return this;
}



Playground* Playground::move(PlaygroundObjectID id, float dx, float dy)
{
    pathfinder_move_zone(playground_impl->my_playground, id, dx, dy);
    return this;
}


Playground* Playground::change_shape(PlaygroundObjectID id, const std::vector<Point>& points)
{
    pathfinder_update_zone(playground_impl->my_playground, id, points);
    return this;
}


Playground* Playground::get_shape(std::vector<Point> * & points, PlaygroundObjectID id)
{
    points = pathfinder_get_zone(playground_impl->my_playground, id);
    return this;
}


Playground* Playground::enable(PlaygroundObjectID id, bool is_enabled)
{
    pathfinder_set_is_enabled_zone(playground_impl->my_playground, id, is_enabled);
    return this;
}


Playground* Playground::detect(PlaygroundObjectID id, bool is_detected)
{
    pathfinder_set_is_detected_zone(playground_impl->my_playground, id, is_detected);
    return this;
}

Playground* Playground::compute_edges()
{
    pathfinder_field_config_done(playground_impl->my_playground);
    return this;
}


void Playground::PlaygroundImpl::enable(PlaygroundObjectID teammate, PlaygroundObjectID opponent_1, PlaygroundObjectID opponent_2, int enabled)
{
    if (teammate != Playground::INVALID && pathfinder_is_detected_zone(my_playground, teammate) && pathfinder_is_enabled_zone(my_playground, teammate)) {
        pathfinder_enable_zone(my_playground, teammate, enabled);
    }
    if (opponent_1 != Playground::INVALID && pathfinder_is_detected_zone(my_playground, opponent_1) && pathfinder_is_enabled_zone(my_playground, opponent_1)) {
        pathfinder_enable_zone(my_playground, opponent_1, enabled);
    }
    if (opponent_2 != Playground::INVALID && pathfinder_is_detected_zone(my_playground, opponent_2) && pathfinder_is_enabled_zone(my_playground, opponent_2)) {
        pathfinder_enable_zone(my_playground, opponent_2, enabled);
    }
}


Playground* Playground::find_path(FoundPath * & path, Point& start, Point& end)
{
    playground_impl->enable(teammate, opponent_1, opponent_2, 1);
    path = pathfinder_find_path(playground_impl->my_playground, start.x, start.y, end.x, end.y);
    playground_impl->enable(teammate, opponent_1, opponent_2, 0);
//    logger.log("Compute route from ({}, {}) to ({}, {})".format(start.x, start.y, end.x, end.y))
//    start_date = datetime.datetime.now()
//    (cost, path) = self.pathfinder.find_path(start.x, start.y, end.x, end.y)
//    delta = datetime.datetime.now() - start_date
//    if len(path) == 0:
//        logger.log("No route found. Cost: {}. computation time: {}".format(cost, delta.total_seconds()))
//        return None, []
//    else:
//        logger.log("Route computed. Cost: {}. computation time: {}".format(cost, delta.total_seconds()))
    return this;
}

unsigned int Playground::get_nodes_count()
{
    return playground_impl->my_playground->nodes.size();
}

unsigned int Playground::get_nodes_count(PlaygroundObjectID id)
{
    return playground_impl->my_playground->zones[id]->nodes_count;
}

unsigned int Playground::get_edges_count()
{
    return playground_impl->my_playground->edges.size();
}


unsigned int Playground::get_zones_count()
{
    return playground_impl->my_playground->zones.size();
}
