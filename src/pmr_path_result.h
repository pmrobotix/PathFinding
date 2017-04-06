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
};

#endif /* PMR_PATH_RESULT_H_ */
