#include <cstddef>
#include <vector>
#include <exception>

#include "pmr_path_result.h"
#include "pmr_pathfinding.h"
#include "pmr_zone.h"
#include "pmr_tools.h"
#include "pmr_debug_logging.h"


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
    PMR_DBG("pathfinder_init");
    self->field_x1 = field_x1;
    self->field_y1 = field_y1;
    self->field_x2 = field_x2;
    self->field_y2 = field_y2;
    self->zone_escape_increment = zone_escape_increment;
    self->zone_escape_max_increment = zone_escape_max_increment;

    self->is_field_config_done = 0;
    self->is_synchronized = 0;

    /* Field nodes */
    self->start_node = node_new(0.0, 0.0);
    self->nodes.push_back(self->start_node);
    self->end_node = node_new(0.0, 0.0);
    self->nodes.push_back(self->end_node);

    return 0;
}


void pathfinder_dealloc(PathFinder* self)
{
    PMR_DBG("pathfinder_dealloc");
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
    PMR_DBG("New zone with center " << x << ", " << y << " and ID " << zone->id);

    for (i = 0; i < zone->nodes_count; ++i) {
        self->nodes.push_back((zone->nodes)[i]);
        PMR_DBG("Zone " << zone->id << ":" << i << ": Node " << (zone->nodes)[i]->x << ", " << (zone->nodes)[i]->y);
    }

    return zone->id;
}


void pathfinder_enable_zone(PathFinder* self, unsigned int zone_id, int enabled)
{

    self->is_synchronized = 0;

    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        self->zones[(size_t)zone_id]->enabled = enabled;
        PMR_DBG("Enable zone " << zone_id << ", value " << enabled);
    }
    else {
        PMR_DBG("ERROR: failure in Enable zone " << zone_id);
    }

}


void pathfinder_set_is_enabled_zone(PathFinder* self, unsigned int zone_id, bool is_enabled)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        zone_set_is_enabled(self->zones[(size_t)zone_id], is_enabled);
        PMR_DBG("Set enable zone " << zone_id << ", value " << is_enabled);
    }
    else {
        PMR_DBG("ERROR: failure in Set enable zone (not existing) " << zone_id);
    }
}


void pathfinder_set_is_detected_zone(PathFinder* self, unsigned int zone_id, bool is_detected)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        zone_set_is_detected(self->zones[(size_t)zone_id], is_detected);
        PMR_DBG("Set detected zone " << zone_id << ", value " << is_detected);
    }
    else {
        PMR_DBG("ERROR: failure in Set detected zone (not existing) " << zone_id);
    }

}


bool pathfinder_is_enabled_zone(PathFinder* self, unsigned int zone_id)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        PMR_DBG("Is zone enabled?" << zone_id << ", " << zone_is_enabled(self->zones[(size_t)zone_id]));
        return zone_is_enabled(self->zones[(size_t)zone_id]);
    }
    PMR_DBG("ERROR: failure in is enabled zone (not existing) " << zone_id);
    return false;
}


bool pathfinder_is_detected_zone(PathFinder* self, unsigned int zone_id)
{
    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        PMR_DBG("Is zone detected?" << zone_id << ", " << zone_is_detected(self->zones[(size_t)zone_id]));
        return zone_is_detected(self->zones[(size_t)zone_id]);
    }
    PMR_DBG("ERROR: failure in is detected zone (not existing) " << zone_id);
    return false;
}


void pathfinder_move_zone(PathFinder* self, unsigned int zone_id, float dx, float dy)
{
    self->is_synchronized = 0;

    if (zone_id >= 0 && zone_id < (self->zones.size())) {
        Zone* zone = self->zones[(size_t) zone_id];
        zone->dx += dx;
        zone->dy += dy;
        PMR_DBG(zone_id << ": zone moved by " << dx << ", " << dy);
    }
    else {
        PMR_DBG("ERROR: failure in move zone (not existing) " << zone_id);
    }

}


void pathfinder_update_zone(PathFinder* self, unsigned int zone_id, const std::vector<Point>& points_list)
{

    self->is_synchronized = 0;

    if (zone_id >= 0 && zone_id < (size_t)(self->zones.size())) {
        Zone* zone = self->zones[zone_id];
        zone_update(zone, points_list);
        PMR_DBG(zone_id << ": zone updated");
    }
    else {
        PMR_DBG("ERROR: failure in update zone (not existing) " << zone_id);
    }
}


std::vector<Point> * pathfinder_get_zone(PathFinder* self, unsigned int zone_id)
{
    if (zone_id >= 0 && zone_id < (size_t)(self->zones.size())) {
        Zone* zone = self->zones[zone_id];
        unsigned int size = zone->nodes_count;
        std::vector<Point> * result = new std::vector<Point>(size);
        PMR_DBG(zone_id << ": get zone with points:");
        for (unsigned int i = 0; i < size; i++) {
            Point p;
            p.x = zone->nodes[i]->x;
            p.y = zone->nodes[i]->y;
            PMR_DBG(zone_id << ":" << i << ": Point " << p.x << ", " << p.y);
            (*result)[i] = p;
        }
        return result;
    }
    PMR_DBG("ERROR: failure in get zone (not existing) " << zone_id);
    return NULL;
}


std::vector<Edge*>* pathfinder_get_edges(PathFinder* self)
{
    PMR_DBG("Get edges");
    return &(self->edges);
}


static Edge* pathfinder_fetch_edge(PathFinder* self, Node* node1, Node* node2)
{
    Edge* edge = NULL;
    std::vector<Edge*>::iterator edges_it;

    for (edges_it = self->edges.begin(); edges_it < self->edges.end(); edges_it++) {
        edge = *edges_it;
        if (edge_links(edge, node1, node2)) {
            PMR_DBG("Fetch edge (existing) node 1: " << node1->x << ", " << node1->y << " node 2: " << node2->x << ", " << node2->y);
            return edge;
        }
    }
    edge = edge_new(node1, node2);
    self->edges.push_back(edge);
    PMR_DBG("Fetch edge (new) node 1: " << node1->x << ", " << node1->y << " node 2: " << node2->x << ", " << node2->y);

    return edge;
}


static int pathfinder_is_node_in_field(PathFinder* self, Node* node)
{
    int result = tools_is_between(self->field_x1, self->field_x2, node->x) &&
            tools_is_between(self->field_y1, self->field_y2, node->y);
    PMR_DBG("Is node in field node: " << node->x << ", " << node->y << ": " << result);
    return result;
}


static void pathfinder_update_edges_affine_params(PathFinder* self, Edge** edges, int edges_count)
{
    int i = 0;

    PMR_DBG("Update edges affine params: " << edges_count);
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

    PMR_DBG("Disable intersecting edges: " << edges_count);
    for (i = 0; i < edges_count; ++i) {
        Edge* edge1 = edges[i];
        if (edge1->enabled) {
            PMR_DBG("Checking if enabled edge Node 1:"  <<edge1->node1->x << ", " << edge1->node1->y << " Node 2: " << edge1->node2->x << ", " << edge1->node2->y);
            for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
                Zone* zone = *zones_it;
                if (zone->enabled) {
                    PMR_DBG("Is it in zone " << zone->id);
                    for (k = 0; k < zone->nodes_count; ++k) {
                        Edge* edge2 = zone->edges[k];
                        int semi_enabled = (!edge2->zone_internal) && (edge2->node1->enabled || edge2->node2->enabled);
                        if (edge1 != edge2 && semi_enabled && edge_intersects(edge1, edge2)) {
                            edge1->enabled = 0;
                            PMR_DBG("---> disabled");
                            break;
                        }
                    }
                }
            }
        }
    }
}


void pathfinder_synchronize(PathFinder* self)
{
    std::vector<Zone*>::iterator zones_it;

    PMR_DBG("Synchronize");
    PMR_DBG("Start node: " << self->start_node->x << ", " << self->start_node->y);
    PMR_DBG("End node: " << self->end_node->x << ", " << self->end_node->y);
    if (self->is_synchronized) {
        PMR_DBG("Synchronize, was synchronized");
        self->end_node->enabled = pathfinder_is_node_in_field(self, self->end_node);
        if (self->end_node->enabled) {
            /* Disable end node if it is in a zone*/
            for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
                Zone* zone = *zones_it;
                if (zone->enabled && zone_contains_node(zone, self->end_node)) {
                    self->end_node->enabled = 0;
                    PMR_DBG("End node disabled because it is in zone " << zone->id);
                    break;
                }
            }
        }

        /* Update edge affine params */
        PMR_DBG("Synchronize start and end node");
        pathfinder_update_edges_affine_params(self, self->end_node->edges, self->end_node->edges_count);
        pathfinder_update_edges_affine_params(self, self->start_node->edges, self->start_node->edges_count);

        /* Remove intersecting edges */
        pathfinder_disable_intersecting_edges(self, self->end_node->edges, self->end_node->edges_count);
        pathfinder_disable_intersecting_edges(self, self->start_node->edges, self->start_node->edges_count);
    } else {
        PMR_DBG("Synchronize, was not synchronized");
        /* Apply zone translations */
        std::vector<Node*>::iterator nodes_it;

        for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
            Zone* zone = *zones_it;
            if (zone->dx != 0.0 || zone->dy != 0.0) {
                for (unsigned int j = 0; j < zone->nodes_count; ++j) {
                    Node* node = zone->nodes[j];
                    PMR_DBG(zone->id << ":" << j << ": old node coord: " << node->x << ", " << node->y);
                    node->x += zone->dx;
                    node->y += zone->dy;
                    PMR_DBG(zone->id << ":" << j << ": new node coord: " << node->x << ", " << node->y);
                }
            }
            zone->dx = 0.0;
            zone->dy = 0.0;
        }
        /* Remove nodes outside of field */
        PMR_DBG("Remove nodes outside of field");
        for (nodes_it = self->nodes.begin(); nodes_it < self->nodes.end(); nodes_it++) {
            Node* node = *nodes_it;
            node->enabled = pathfinder_is_node_in_field(self, node);
        }
        /* Remove disabled zones nodes */
        PMR_DBG("Remove nodes from disabled zones");
        for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
            Zone* zone = *zones_it;
            for (unsigned int j = 0; j < zone->nodes_count; ++j) {
                Node* node = zone->nodes[j];
                node->enabled &= zone->enabled;
            }
        }
        /* Remove nodes in a zone*/
        PMR_DBG("Remove nodes inside a zone");
        for (nodes_it = self->nodes.begin(); nodes_it < self->nodes.end(); nodes_it++) {
            Node* node = *nodes_it;
            if (node->enabled) {
                for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
                    Zone* zone = *zones_it;
                    if (zone->enabled && zone_contains_node(zone, node)) {

                        node->enabled = 0;
                        PMR_DBG(zone->id << ":" << ": disabled node: " << node->x << ", " << node->y);
                        break;
                    }
                }
            }
        }
        /* Start node is always enabled */
        self->start_node->enabled = 1;

        /* Update edge affine params */
        PMR_DBG("Synchronize all edges");
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

    PMR_DBG("Field config done");
    if (self->is_field_config_done) {
        return;
    }

    /* Number of edges in a graph of N nodes: N * (N - 1) / 2 */
    /* max_edges = self->nodes.size() * (self->nodes.size() - 1) / 2; */
    /* Create edges arrays on nodes */
    PMR_DBG("Creating edges: " << self->nodes.size() << " * " << (self->nodes.size()-1));
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
    PMR_DBG("Find border edges for each zone (no new edges should be created here)");
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
                PMR_DBG(zone->id << ":" << ": zone-internal edge : " << edge->node1->x << ", " << edge->node1->y << " and " << edge->node2->x << ", " << edge->node2->y);
                break;
            }
        }
    }

    pathfinder_synchronize(self);

    self->is_field_config_done = 1;

}


static int pathfinder_move_node_outside_of_zone(PathFinder* self, Zone* zone, Node* node)
{
    float center_x = 0.0;
    float center_y = 0.0;
    float orig_x = node->x;
    float orig_y = node->y;
    float length = 0.0;
    float increment = 0.0;

    PMR_DBG("Move node outside of zone " << zone->id << " for node:" << node->x << ", " << node->y);
    if (!zone_contains_node(zone, node)) {
        return 0;
    }

    PMR_DBG("Node must be moved!");
    zone_get_center(zone, &center_x, &center_y);
    length = tools_distance(center_x, center_y, node->x, node->y);
    do {
        float ratio = 0.0;
        increment += self->zone_escape_increment;
        ratio = (length + increment) / length;
        node->x = ratio * (orig_x - center_x) + center_x;
        node->y = ratio * (orig_y - center_y) + center_y;
        PMR_DBG("New node coords:" << node->x << ", " << node->y);
    } while(zone_contains_node(zone, node) && increment <= self->zone_escape_max_increment);

    return 1;
}


float pathfinder_effective_cost(PathFinder* self, Edge* edge)
{
    PMR_DBG("Effective cost of edge with node1: " << edge->node1->x << ", " << edge->node1->y << " node2: " << edge->node2->x << ", " << edge->node2->y << " is " << edge->length);
    return edge->length;
}


float pathfinder_heuristic_cost_estimate(PathFinder* self, Node* neighbor)
{
    Node* last_node = self->end_node;
    float result = tools_distance(neighbor->x, neighbor->y, last_node->x, last_node->y);
    PMR_DBG("Estimated code of node: " << neighbor->x << ", " << neighbor->y << " nodeEnd: " << last_node->x << ", " << last_node->y << " is " << result);
    return result;
}


FoundPath* pathfinder_find_path(PathFinder* self, float x1, float y1, float x2, float y2)
{
    unsigned int i;
    Node* openset = self->start_node;
    Node* last_node = self->end_node;

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
    PMR_DBG("[P]Find path");
    PMR_DBG("[P]Start node: " << self->start_node->x << ", " << self->start_node->y);
    PMR_DBG("[P]End node: " << self->end_node->x << ", " << self->end_node->y);

    pathfinder_synchronize(self);
    PMR_DBG("[P]Check if start path in zone. Start");
    for (zones_it = self->zones.begin(); zones_it < self->zones.end(); zones_it++) {
        Zone* zone = *zones_it;
        if (zone->enabled && pathfinder_move_node_outside_of_zone(self, zone, self->start_node)) {
            pathfinder_synchronize(self);
            break;
        }
    }
    PMR_DBG("[P]Check if start path in zone. End");

    PMR_DBG("[P]Reset all nodes");
    for (nodes_it = self->nodes.begin(); nodes_it < self->nodes.end(); nodes_it++) {
        Node* n = *nodes_it;
        n->is_in_openset = 0;
        n->is_in_closedset = 0;
        n->g_score = 0.0;
        n->f_score = 0.0;
        n->h_score = 0.0;
        n->came_from = NULL;
        n->next = NULL;
    }

    self->start_node->g_score = 0.0;
    self->start_node->f_score = 0.0;
    self->start_node->h_score = 0.0;
    self->start_node->came_from = NULL;
    self->start_node->next = NULL;
    self->start_node->is_in_openset = 1;

    while (openset != NULL) {
        Node* current = openset;
        PMR_DBG("[P]Current node: " << current->x << ", " << current->y);
        openset = node_list_pop(openset);
        if (node_coords_equal(current, self->end_node)) {
            float cost = current->f_score;
            std::vector<Node*> path;
            for (; current != NULL; current = current->came_from) {
                cost += current->f_score;
                node = node_new(current->x, current->y);
                path.push_back(node);
            }
            result->cost = cost;
            PMR_DBG("[P]Path to end node found! The cost is " << result->cost);
            for (nodes_rit = path.rbegin(); nodes_rit < path.rend(); nodes_rit++) {
                Node* n = *nodes_rit;
                result->path.push_back(n);
                PMR_DBG("[P]Next result node: " << n->x << ", " << n->y);
            }
            return result;
        }
        current->is_in_openset = 0;
        current->is_in_closedset = 1;

        for (i = 0; i < current->edges_count; ++i) {
            Edge* edge = current->edges[i];
            if (edge->enabled) {
                PMR_DBG("[P]Enabled edge with node1: " << edge->node1->x << ", " << edge->node1->y << " node2: " << edge->node2->x << ", " << edge->node2->y);
                Node* neighbor = edge_other_node(edge, current);
                PMR_DBG("[P]Neighbor is: " << neighbor->x << ", " << neighbor->y);
                if (!neighbor->is_in_closedset) {
                    float tentative_g_score = current->g_score + pathfinder_effective_cost(self, edge);
                    PMR_DBG("[P]Edge is not yet explored! Tentative g_score: " << tentative_g_score);
                    if (!neighbor->is_in_openset) {
                        neighbor->h_score = pathfinder_heuristic_cost_estimate(self, neighbor);
                        neighbor->g_score = tentative_g_score;
                        neighbor->f_score = neighbor->g_score + neighbor->h_score;
                        neighbor->came_from = current;
                        openset = node_list_insert_sorted(openset, neighbor);
                        neighbor->is_in_openset = 1;
                        PMR_DBG("[P]Edge is new. f_score: " << neighbor->f_score);
                    } else if (tentative_g_score < neighbor->g_score) {
                        neighbor->g_score = tentative_g_score;
                        neighbor->f_score = neighbor->g_score + neighbor->h_score;
                        neighbor->came_from = current;
                        PMR_DBG("[P]Edge is known. f_score: " << neighbor->f_score);
                    }
                }
            }
        }
    }
    PMR_DBG("[P]No result found! Returning new node 0, 0 as result");
    node = node_new(0.0, 0.0);
    result->path.push_back(node);
    return result;
}
