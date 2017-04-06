/*
 * pmr_node.cpp
 *
 *  Created on: Apr 6, 2017
 *      Author: gmo
 */

#include <cstddef>
#include "pmr_node.h"
#include "pmr_tools.h"


Node* node_new(float x, float y)
{
    Node* self = new Node;
    self->x = x;
    self->y = y;
    self->g_score = 0.0;
    self->h_score = 0.0;
    self->f_score = 0.0;
    self->edges = NULL;
    self->edges_count = 0;
    self->is_in_openset = 0;
    self->is_in_closedset = 0;
    self->enabled = 1;
    self->next = NULL;
    self->came_from = NULL;
    return self;
}


void node_free(Node* self)
{
    if (self->edges != NULL) {
        delete self->edges;
    }
    delete self;
}


void node_create_edges_array(Node* self, unsigned int size)
{
    self->edges = new Edge*[size];
}


void node_add_edge(Node* self, Edge* edge)
{
    self->edges[self->edges_count++] = edge;
}


int node_coords_equal(Node* self, Node* other)
{
    return tools_quasi_equal(self->x, other->x) && tools_quasi_equal(self->y, other->y);
}


Node* node_list_insert_sorted(Node* self, Node* other)
{
    Node* previous = NULL;
    Node* current = NULL;

    /* The list is empty */
    if (self == NULL) {
        other->next = NULL;
        return other;
    }

    /* Middle of the list */
    for (current = self; current != NULL; current = current->next) {
        if (current->f_score >= other->f_score) {
            if (previous != NULL) {
                previous->next = other;
                other->next = current;
                return self;
            } else {
                other->next = self;
                return other;
            }
        }
        previous = current;
    }

    /* other is greater than all list items */
    previous->next = other;
    other->next = NULL;
    return self;
}


Node* node_list_pop(Node* self)
{
    Node* popped = self;
    self = self->next;
    popped->next = NULL;
    return self;
}

