/*
 * pmr_debug_logging.h
 *
 *  Created on: Apr 26, 2017
 *      Author: gmo
 */

#ifndef PMR_DEBUG_LOGGING_H_
#define PMR_DEBUG_LOGGING_H_

#ifdef PMR_DEBUG_ENABLED
#include <iostream>

#define PMR_DBG(msg) \
    std::cout << __FILE__ << "(" << __LINE__ << "): " << msg << std::endl
#else /* PMR_DEBUG_ENABLED */
#define PMR_DBG(msg)
#endif /* PMR_DEBUG_ENABLED */



#endif /* PMR_DEBUG_LOGGING_H_ */
