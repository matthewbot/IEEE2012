#ifndef NAV_H
#define NAV_H

void nav_magGo(float heading_deg, float dist);

bool nav_linefollowIntersection();
bool nav_linefollowTurns(int turncount, float offset=0);
bool nav_linefollowRange(float range);
bool nav_linefollowDist(float dist);
bool nav_linefollow(float offset=0);

void nav_pause();
void nav_setPauseEnabled(bool pause);

#endif
