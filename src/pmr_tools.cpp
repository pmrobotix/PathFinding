#include "pmr_tools.h"
#include <cmath>


/*****************************************************************************/
/* General purpose features                                                  */
/*****************************************************************************/

/* Tools */


int tools_quasi_equal(float a, float b)
{
    return fabs(a - b) < TOOLS_EPSILON;
}


int tools_is_between(float a, float b, float x)
{
    if (tools_quasi_equal(a, x) || tools_quasi_equal(b, x)) {
        return 1;
    }
    if (a < b) {
        return a < x && x < b;
    } else {
        return b < x && x < a;
    }
}


float tools_distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}
