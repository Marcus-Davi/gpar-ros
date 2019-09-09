

#ifndef WAYPOINTLOADER_H
#define WAYPOINTLOADER_H

#include <ros/ros.h>

#include <djiosdk/dji_vehicle.hpp>

#include <fstream>


std::vector<DJI::OSDK::WayPointSettings>* WaypointLoad(const char* file);




#endif //WAYPOINTSLOADER_H



