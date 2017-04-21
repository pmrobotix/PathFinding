#include <cstddef>
#include <vector>
#include <exception>

#include "pmr_path_result.h"
#include "pmr_pathfinding.h"
#include "pmr_zone.h"
#include "pmr_tools.h"


/* Exceptions */
class RuntimeErrorZoneSetupAlreadyFinished: public std::exception
{
    virtual const char* what() const throw() {
        return "Setup already finished. Adding new zones is forbidden";
    }
};

RuntimeErrorZoneSetupAlreadyFinished zoneSetupFinished;


/*****************************************************************************/
/* Path finding code                                                         */
/*****************************************************************************/


/* Pathfinder methods ********************************************************/

int pathfinder_init(PathFinder* self, float field_x1, float field_y1, float field_x2, float field_y2,
        float zone_escape_increment, float zone_escape_max_increment)
{
    self->field_x1 = field_x1;
    self->field_y1 = field_y1;
    self->field_x2 = field_x2;
    self->field_y2 = field_y2;
    self->zone_escape_increment = zone_escape_increment;
    self->zone_escape_max_increment = zone_escape_max_increment;

    self->is_field_config_done = 0;
    self->is_synchronized = 0;

    /* Field nodes */
    self->nodes.push_back(node_new(0.0, 0.0));
    self->nodes.push_back(node_new(0.0, 0.0));

    return 0;
}


void pathfinder_dealloc(PathFinder* self)
{
    std::vector<Node*>::reverse_iterator nodes_it;
    std::vector<Zone*>::reverse_iterator zones_it;
    std::vector<Edge*>::reverse_iterator edges_it;

    for (nodes_it = self->nodes.rbegin(); nodes_it < self->nodes.rend(); nodes_it++) {
        node_free(*nodes_it);
    }
    self->nodes.clear();

    for (zones_it = self->zones.rbegin(); zones_it < self->zones.rend(); zones_it++) {
        zone_free(*zones_it);
    }
    self->zones.clear();

    for (edges_it = self->edges.rbegin(); edges_it < self->edges.rend(); edges_it++) {
        edge_free(*edges_it);
    }
    self->edges.clear();

}


unsigned int pathfinder_add_zone(PathFinder* self, std::vector<Point>& points_list)
{
    unsigned int i = 0;

    self->is_synchronized = 0;

    if (self->is_field_config_done) {
        throw zoneSetupFinished;
    }

    Zone* zone = zone_new((unsigned int)(self->zones.size()), points_list);
    float x, y;
    zone_get_center(zone, &x, &y);
    self->zones.push_back(zone);

    for (i = 0; i < zone->nodes_count; ++i) {
        self->nodes.push_back((zone->nodes)[i]);
    }

    return zone->id;
}


void pathfinder_enable_zone(PathFinder* self, unsigned int zone_id, int enabled)
{

    self->is_synchronized = 0;

    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        self->zones[(size_t)zone_id]->enabled = enabled;
    }

}


void pathfinder_set_is_enabled_zone(PathFinder* self, unsigned int zone_id, bool is_enabled)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        zone_set_is_enabled(self->zones[(size_t)zone_id], is_enabled);
    }
}


void pathfinder_set_is_detected_zone(PathFinder* self, unsigned int zone_id, bool is_detected)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        zone_set_is_detected(self->zones[(size_t)zone_id], is_detected);
    }
}


bool pathfinder_is_enabled_zone(PathFinder* self, unsigned int zone_id)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        return zone_is_enabled(self->zones[(size_t)zone_id]);
    }
    return false;
}


bool pathfinder_is_detected_zone(PathFinder* self, unsigned int zone_id)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        return zone_is_detected(self->zones[(size_t)zone_id]);
    }
    return false;
}


void pathfinder_move_zone(PathFinder* self, unsigned int zone_id, float dx, float dy)
{
    self->is_synchronized = 0;

    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        Zone* zone = self->zones[(size_t) zone_id];
        zone->dx += dx;
        zone->dy += dy;
    }

}


void pathfinder_update_zone(PathFinder* self, unsigned int zone_id, const std::vector<Point>& points_list)
{

    self->is_synchronized = 0;

    if (zone_id >= 0 && zone_id < (size_t)(self->zones.size())) {
        Zone* zone = self->zones[zone_id];
        return zone_update(zone, points_list);
    }

}


std::vector<Point> * pathfinder_get_zone(PathFinder* self, unsigned int zone_id)
{
    if (zone_id >= 0 && zone_id < (size_t)(self->zones.size())) {
        Zone* zone = self->zones[zone_id];
        unsigned int size = zone->nodes_count;
        std::vector<Point> * result = new std::vector<Point>(size);
        for (unsigned int i = 0; i < size; i++) {
            Point p;
            p.x = zone->nodes[i]->x;
            p.y = zone->nodes[i]->y;
            (*result)[i] = p;
        }
        return result;
    }
    return NULL;
}


std::vector<Edge*>* pathfinder_get_edges(PathFinder* self)
{
    return &(self->edges);
}


static Edge* pathfinder_fetch_edge(PathFinder* self, Node* node1, Node* node2)
{
    Edge* edge = NULL;
    std::vector<Edge*>::iterator edges_it;

    for (edges_it = self->edges.begin(); edges_it < self->edges.end(); edges_it++) {
        edge = *edges_it;
        if (edge_links(edge, node1, node2)) {
            return edge;
        }
    }
    edge = edge_new(node1, node2);
    self->edges.push_back(edge);

    return edge;
}


static int pathfinder_is_node_in_field(PathFinder* self, Node* node)
{
    return tools_is_between(self->field_x1, self->field_x2, node->x) &&
           tools_is_between(self->field_y1, self->field_y2, node->y);
}


static void pathfinder_update_edges_affine_params(PathFinder* self, Edge** edges, int edges_count)
{
    int i = 0;

    for (i = 0; i < edges_count; ++i) {
        Edge* edge = edges[i];
        edge_update(edge);
        edge->enabled = (!edge->zone_internal) && edge->node1->enabled && edge->node2->enabled;
    }
}


static void pathfinder_disable_intersecting_edges(PathFinder* self, Edge** edges, unsigned int edges_count)
{
    unsigned int i = 0;
    std::vector<Zone*>::iterator zones_it;
    unsigned int k = 0;

    for (i = 0; i < edges_count; ++i) {
        Edge* edge1 = edges[i];
        if (edge1->enabled) {
            for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
                Zone* zone = *zones_it;
                if (zone->enabled) {
                    for (k = 0; k < zone->nodes_count; ++k) {
                        Edge* edge2 = zone->edges[k];
                        int semi_enabled = (!edge2->zone_internal) && (edge2->node1->enabled || edge2->node2->enabled);
                        if (edge1 != edge2 && semi_enabled && edge_intersects(edge1, edge2)) {
                            edge1->enabled = 0;
                        }
                    }
                }
            }
        }
    }
}


static void pathfinder_synchronize(PathFinder* self)
{
    std::vector<Zone*>::iterator zones_it;

    Node* start_node = self->nodes[0];
    if (self->is_synchronized) {
        Node* end_node = self->nodes.back();
        end_node->enabled = pathfinder_is_node_in_field(self, end_node);
        if (end_node->enabled) {
            /* Disable end node if it is in a zone*/
            for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
                Zone* zone = *zones_it;
                if (zone->enabled && zone_contains_node(zone, end_node)) {
                    end_node->enabled = 0;
                    break;
                }
            }
        }

        /* Update edge affine params */
        pathfinder_update_edges_affine_params(self, end_node->edges, end_node->edges_count);
        pathfinder_update_edges_affine_params(self, start_node->edges, start_node->edges_count);

        /* Remove intersecting edges */
        pathfinder_disable_intersecting_edges(self, end_node->edges, end_node->edges_count);
        pathfinder_disable_intersecting_edges(self, start_node->edges, start_node->edges_count);
    } else {
        /* Apply zone translations */
        std::vector<Node*>::iterator nodes_it;

        for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
            Zone* zone = *zones_it;
            for (unsigned int j = 0; j < zone->nodes_count; ++j) {
                Node* node = zone->nodes[j];
                node->x += zone->dx;
                node->y += zone->dy;
            }
            zone->dx = 0.0;
            zone->dy = 0.0;
        }
        /* Remove nodes outside of field */
        for (nodes_it = self->nodes.begin(); nodes_it < self->nodes.end(); nodes_it++) {
            Node* node = *nodes_it;
            node->enabled = pathfinder_is_node_in_field(self, node);
        }
        /* Remove disabled zones nodes */
        for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
            Zone* zone = *zones_it;
            for (unsigned int j = 0; j < zone->nodes_count; ++j) {
                Node* node = zone->nodes[j];
                node->enabled &= zone->enabled;
            }
        }
        /* Remove nodes in a zone*/
        for (nodes_it = self->nodes.begin(); nodes_it < self->nodes.end(); nodes_it++) {
            Node* node = *nodes_it;
            if (node->enabled) {
                for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
                    Zone* zone = *zones_it;
                    if (zone->enabled && zone_contains_node(zone, node)) {
                        node->enabled = 0;
                        break;
                    }
                }
            }
        }
        /* Start node is always enabled */
        start_node->enabled = 1;

        /* Update edge affine params */
        pathfinder_update_edges_affine_params(self, self->edges.data(), (int) self->edges.size());

        /* Remove intersecting edges */
        pathfinder_disable_intersecting_edges(self, self->edges.data(), (int) self->edges.size());

        self->is_synchronized = 1;
    }
}


void pathfinder_field_config_done(PathFinder* self)
{
    unsigned int i = 0;
    unsigned int j = 0;
    std::vector<Zone*>::iterator zones_it;
    std::vector<Edge*>::iterator edges_it;
    /*int max_edges = 0;b*/

    if (self->is_field_config_done) {
        return;
    }

    /* Number of edges in a graph of N nodes: N * (N - 1) / 2 */
    /* max_edges = self->nodes.size() * (self->nodes.size() - 1) / 2; */
    /* Create edges arrays on nodes */
    for (i = 0; i < self->nodes.size(); ++i) {
        node_create_edges_array(self->nodes[i], self->nodes.size() - 1);
    }
    /* Create all edges */
    for (i = 0; i < self->nodes.size(); ++i) {
        Node* node1 = self->nodes[i];
        for (j = 0; j < self->nodes.size(); ++j) {
            Node* node2 = self->nodes[j];
            if (node1 != node2) {
                Edge* edge = pathfinder_fetch_edge(self, node1, node2);
                node_add_edge(node1, edge);
            }
        }
    }
    /* Register surrounding edges of each zone */
    for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
        Zone* zone = *zones_it;
        Node* previous_node = zone->nodes[zone->nodes_count - 1];
        for (j = 0; j < zone->nodes_count; ++j) {
            Node* node = zone->nodes[j];
            Edge* edge = pathfinder_fetch_edge(self, previous_node, node);
            zone->edges[j] = edge;
            previous_node = node;
        }
    }
    /* Mark zones internal edges */
    for (edges_it = self->edges.begin(); edges_it < self->edges.end(); edges_it++) {
        Edge* edge = *edges_it;
        for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
            Zone* zone = *zones_it;
            if (zone_is_internal_edge(zone, edge)) {
                edge->zone_internal = 1;
                break;
            }
        }
    }

    pathfinder_synchronize(self);

    self->is_field_config_done = 1;

}


static int pathfinder_move_node_ouside_of_zone(PathFinder* self, Zone* zone, Node* node)
{
    float center_x = 0.0;
    float center_y = 0.0;
    float orig_x = node->x;
    float orig_y = node->y;
    float length = 0.0;
    float increment = 0.0;

    if (!zone_contains_node(zone, node)) {
        return 0;
    }

    zone_get_center(zone, &center_x, &center_y);
    length = tools_distance(center_x, center_y, node->x, node->y);
    do {
        float ratio = 0.0;
        increment += self->zone_escape_increment;
        ratio = (length + increment) / length;
        node->x = ratio * (orig_x - center_x) + center_x;
        node->y = ratio * (orig_y - center_y) + center_y;
    } while(zone_contains_node(zone, node) && increment <= self->zone_escape_max_increment);

    return 1;
}


float pathfinder_effective_cost(PathFinder* self, Edge* edge)
{
    return edge->length;
}


float pathfinder_heuristic_cost_estimate(PathFinder* self, Node* neighbor)
{
    Node* last_node = self->nodes.back();
    return tools_distance(neighbor->x, neighbor->y, last_node->x, last_node->y);
}


FoundPath* pathfinder_find_path(PathFinder* self, float x1, float y1, float x2, float y2)
{
    unsigned int i;
    Node* openset = self->nodes[0];
    Node* last_node = self->nodes.back();
    FoundPath* result = new FoundPath();
    Node* node;
    std::vector<Zone*>::iterator zones_it;
    std::vector<Node*>::iterator nodes_it;
    std::vector<Node*>::reverse_iterator nodes_rit;

    result->cost = 0.0;

    openset->x = x1;
    openset->y = y1;
    last_node->x = x2;
    last_node->y = y2;

    pathfinder_synchronize(self);
    for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
        Zone* zone = *zones_it;
        if (zone->enabled && pathfinder_move_node_ouside_of_zone(self, zone, self->nodes[0])) {
            pathfinder_synchronize(self);
            break;
        }
    }

    for (nodes_it = self->nodes.begin(); nodes_it < self->nodes.end(); nodes_it++) {
        Node* node = *nodes_it;
        node->is_in_openset = 0;
        node->is_in_closedset = 0;
    }
    (self->nodes)[0]->g_score = 0.0;
    (self->nodes)[0]->f_score = 0.0;
    (self->nodes)[0]->h_score = 0.0;
    (self->nodes)[0]->came_from = NULL;
    (self->nodes)[0]->next = NULL;
    (self->nodes)[0]->is_in_openset = 1;

    while (openset != NULL) {
        Node* current = openset;
        openset = node_list_pop(openset);
        if (current == self->nodes.back()) {
            float cost = current->f_score;
            std::vector<Node*> path;
            for (; current != NULL; current = current->came_from) {
                cost += current->f_score;
                node = node_new(current->x, current->y);
                path.push_back(node);
            }
            result->cost = cost;
            for (nodes_rit = path.rbegin(); nodes_rit < path.rend(); nodes_rit++) {
                Node* node = *nodes_rit;
                result->path.push_back(node);
            }
            return result;
        }
        current->is_in_openset = 0;
        current->is_in_closedset = 1;

        for (i = 0; i < current->edges_count; ++i) {
            Edge* edge = current->edges[i];
            if (edge->enabled) {
                Node* neighbor = edge_other_node(edge, current);
                if (!neighbor->is_in_closedset) {
                    float tentative_g_score = current->g_score + pathfinder_effective_cost(self, edge);
                    if (!neighbor->is_in_openset) {
                        neighbor->h_score = pathfinder_heuristic_cost_estimate(self, neighbor);
                        neighbor->g_score = tentative_g_score;
                        neighbor->f_score = neighbor->g_score + neighbor->h_score;
                        neighbor->came_from = current;
                        openset = node_list_insert_sorted(openset, neighbor);
                        neighbor->is_in_openset = 1;
                    } else if (tentative_g_score < neighbor->g_score) {
                        neighbor->g_score = tentative_g_score;
                        neighbor->f_score = neighbor->g_score + neighbor->h_score;
                        neighbor->came_from = current;
                    }
                }
            }
        }
    }
    node = node_new(0.0, 0.0);
    result->path.push_back(node);
    return result;
}
