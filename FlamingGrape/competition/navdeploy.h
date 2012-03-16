#ifndef NAVDEPLOY_H
#define NAVDEPLOY_H

void navdeploy_lap();

void navdeploy_deploy(bool lastbox=false);
bool navdeploy_aroundBox(bool same);

bool navdeploy_loopback();
bool navdeploy_middle(bool same);
void navdeploy_end(bool same);

#endif
