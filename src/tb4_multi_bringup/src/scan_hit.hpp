#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace tb4 {
// Scan endpoints are matched to occupied cell centres using metric distance.
// A four-neighbour distance field overestimates diagonal distances and can
// reject an aligned scan solely because of the map grid's orientation.
inline bool near_occupied_cell(const std::vector<int8_t>& cells,int width,int height,
    int x,int y,double resolution,double radius) {
  if(width<=0||height<=0||cells.size()!=size_t(width)*height||
      !std::isfinite(resolution)||resolution<=0||!std::isfinite(radius)||radius<0)return false;
  const int rx=int(std::min(double(width),std::ceil(radius/resolution)));
  const int ry=int(std::min(double(height),std::ceil(radius/resolution)));
  // SLAM rasters can end at a wall. A return just beyond the raster edge
  // still agrees with an occupied cell inside the same metric tolerance.
  if (x < -rx || y < -ry || x >= width+rx || y >= height+ry) return false;
  for(int yy=std::max(0,y-ry);yy<=std::min(height-1,y+ry);++yy)
    for(int xx=std::max(0,x-rx);xx<=std::min(width-1,x+rx);++xx)
      if(cells[yy*width+xx]>=65&&std::hypot(double(xx)-x,double(yy)-y)*resolution<=radius+1e-12)
        return true;
  return false;
}
}
