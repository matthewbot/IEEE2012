#ifndef NAV_H
#define NAV_H

void nav_magGo(float heading_deg, float dist);

bool nav_linefollowIntersection();
bool nav_linefollowTurns(int turncount);

#endif
