

#include "gpar_dji/waypoints_loader.h"



static void setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 1;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

  
std::vector<WayPointSettings>* WaypointLoad(const char* file_path)
{
 std::ifstream myfile;
 myfile.open(file_path, std::ios::in);
 std::string line;
 int n_wp = 0;


 if(!myfile.is_open())
   return NULL; //FILE OPEN FAIL

  float lat,lon,alt;
  int yaw;

  std::vector<WayPointSettings>* WptLista = new std::vector<WayPointSettings>;
  WayPointSettings Wpt;


while(!myfile.eof()){

  std::getline(myfile,line);
  size_t WP_pos,Comment_pos;

  WP_pos = line.find("WP");
  Comment_pos = line.find("#");

   // std::cout << line << std::endl;
    if(WP_pos < Comment_pos){
    n_wp++;
    sscanf(line.c_str(),"%*s %f %f %f %d %*s",&lat,&lon,&alt,&yaw);
    Wpt.index = n_wp;
    Wpt.latitude = lat;
    Wpt.longitude = lon;
    Wpt.yaw = yaw;
    Wpt.altitude = alt;
    WptLista->push_back(Wpt);
     }

}

    if(n_wp < 2 ){
    delete WptLista;
    return NULL;

   }
    myfile.close();
    return WptLista;
}




