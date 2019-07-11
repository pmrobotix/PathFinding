/*
 * pmr_pathfinding.h
 *
 *  Created on: Apr 5, 2017
 *      Author: gmo
 */

#ifndef PMR_PATHFINDING_H_
#define PMR_PATHFINDING_H_

#include "pmr_point.h"
#include "pmr_node.h"
#include "pmr_edge.h"
#include "pmr_zone.h"
#include "pmr_path_result.h"

struct PathFinder
{
    float field_x1;
    float field_y1;
    float field_x2;
    float field_y2;
    int is_field_config_done;
    int is_synchronized;
    float zone_escape_increment;
    float zone_escape_max_increment;
    Node* start_node;
    Node* end_node;
    std::vector<Node*> nodes;
    std::vector<Zone*> zones;
    std::vector<Edge*> edges;
};

/* Pathfinder methods ********************************************************/

int pathfinder_init(PathFinder* self, float field_x1, float field_y1, float field_x2, float field_y2,
        float zone_escape_increment, float zone_escape_max_increment);


void pathfinder_dealloc(PathFinder* self);


unsigned int pathfinder_add_zone(PathFinder* self, std::vector<Point>& points_list);


void pathfinder_enable_zone(PathFinder* self, unsigned int zone_id, int enabled);


void pathfinder_set_is_enabled_zone(PathFinder* self, unsigned int zone_id, bool is_enabled);


void pathfinder_set_is_detected_zone(PathFinder* self, unsigned int zone_id, bool is_detected);


bool pathfinder_is_enabled_zone(PathFinder* self, unsigned int zone_id);


bool pathfinder_is_detected_zone(PathFinder* self, unsigned int zone_id);

void pathfinder_movexy_zone(PathFinder* self, unsigned int zone_id, float x, float y);
void pathfinder_move_zone(PathFinder* self, unsigned int zone_id, float dx, float dy);


void pathfinder_update_zone(PathFinder* self, unsigned int zone_id, const std::vector<Point>& points_list);


std::vector<Point> * pathfinder_get_zone(PathFinder* self, unsigned int zone_id);


std::vector<Edge*>* pathfinder_get_edges(PathFinder* self);


void pathfinder_field_config_done(PathFinder* self);


float pathfinder_effective_cost(PathFinder* self, Edge* edge);


float pathfinder_heuristic_cost_estimate(PathFinder* self, Node* neighbor);


void pathfinder_synchronize(PathFinder* self);


FoundPath* pathfinder_find_path(PathFinder* self, float x1, float y1, float x2, float y2);



#endif /* PMR_PATHFINDING_H_ */
