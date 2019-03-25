#ifndef __LINEGROUP_H__
#define __LINEGROUP_H__

#include "lineedge.h"

namespace vision {

struct LineGroup{
	LineEdge lines[2];
	int points;

    LineGroup() : points(0) { }
};

}  // namespace vision

#endif // __LINEGROUP_H__
