/*
 * pmr_node.h
 *
 *  Created on: Apr 5, 2017
 *      Author: gmo
 */

#ifndef PMR_NODE_H_
#define PMR_NODE_H_

/* Node **********************************************************************/

struct Edge;

struct Node
{
    float x;
    float y;
    float g_score;
    float h_score;
    float f_score;
    Edge** edges;
    unsigned int edges_count;
    int is_in_openset;
    int is_in_closedset;
    int enabled;
    Node* next;
    Node* came_from;
};

Node* node_new(float x, float y);


void node_free(Node* self);


void node_create_edges_array(Node* self, int size);


void node_add_edge(Node* self, Edge* edge);


int node_coords_equal(Node* self, Node* other);


Node* node_list_insert_sorted(Node* self, Node* other);


Node* node_list_pop(Node* self);


#endif /* PMR_NODE_H_ */
