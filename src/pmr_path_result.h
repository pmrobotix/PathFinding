/*
 * path_result.h
 *
 *  Created on: Apr 5, 2017
 *      Author: gmo
 */

#ifndef PMR_PATH_RESULT_H_
#define PMR_PATH_RESULT_H_

#include <vector>
#include "pmr_node.h"
struct FoundPath {
    float cost;
    std::vector<Node*> path;
    ~FoundPath() {
        std::vector<Node *>::const_iterator node_it;

        for (node_it = path.begin(); node_it < path.end(); node_it++) {
            delete *node_it;
        }
    }
};

#endif /* PMR_PATH_RESULT_H_ */
