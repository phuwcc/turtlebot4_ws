#pragma once
// Bounded, read-only coarse-to-fine scan matching used only to seed AMCL.
// A low score or ambiguous second mode produces no seed, never a fake TF.
#include <algorithm>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <vector>
namespace tb4 {
struct ScanPoint {double x,y;};
struct ScanSeed {double x{},y{},yaw{},score{},gap{};bool unique{false};};
inline double wrap(double v){return std::atan2(std::sin(v),std::cos(v));}
inline ScanSeed global_scan_match(int width,int height,double resolution,
    const std::vector<int8_t>& cells,const std::vector<int>& distance,
    const std::vector<ScanPoint>& points,double seconds=.7) {
  using Clock=std::chrono::steady_clock;
  const auto end=Clock::now()+std::chrono::duration<double>(seconds);
  std::vector<ScanSeed> coarse;
  if(width<=0||height<=0||!std::isfinite(resolution)||resolution<=0||cells.size()!=size_t(width)*height||distance.size()!=cells.size()||points.size()<20)return {};
  auto score=[&](double x,double y,double a) {
    const int ix=std::floor(x),iy=std::floor(y);
    if(ix<0||iy<0||ix>=width||iy>=height||cells[iy*width+ix]!=0||distance[iy*width+ix]*resolution<.2)return -1.;
    const double co=std::cos(a)/resolution,si=std::sin(a)/resolution;
    double sum=0;
    for(const auto& p:points) {
      const int xx=std::floor(x+co*p.x-si*p.y),yy=std::floor(y+si*p.x+co*p.y);
      if(xx>=0&&yy>=0&&xx<width&&yy<height) {
        const double d=distance[yy*width+xx]*resolution;
        sum+=std::exp(-d*d/(2*.09*.09));
      }
    }
    return sum/points.size();
  };
  const int stride=std::max(2,int(std::round(.20/resolution)));
  for(int y=0;y<height;y+=stride)for(int x=0;x<width;x+=stride) {
    if(Clock::now()>end)return {};
    if(cells[y*width+x]!=0||distance[y*width+x]*resolution<.2)continue;
    for(int k=0;k<72;++k) {
      const double a=k*2*M_PI/72;const double value=score(x+.5,y+.5,a);
      if(value>.4)coarse.push_back({x+.5,y+.5,a,value,0,false});
    }
  }
  std::sort(coarse.begin(),coarse.end(),[](const auto& a,const auto& b){return a.score>b.score;});
  std::vector<ScanSeed> modes;
  for(const auto& c:coarse) {
    bool separate=true;
    for(const auto& m:modes)if(std::hypot(c.x-m.x,c.y-m.y)*resolution<.35&&std::abs(wrap(c.yaw-m.yaw))<.35){separate=false;break;}
    if(separate)modes.push_back(c);
    if(modes.size()==20)break;
  }
  for(auto& mode:modes) {
    const auto original=mode;
    for(int dy=-stride;dy<=stride;++dy)for(int dx=-stride;dx<=stride;++dx)for(int da=-5;da<=5;++da) {
      if(Clock::now()>end)return {};
      const double a=original.yaw+da*M_PI/180;
      const double value=score(original.x+dx,original.y+dy,a);
      if(value>mode.score)mode={original.x+dx,original.y+dy,a,value,0,false};
    }
  }
  std::sort(modes.begin(),modes.end(),[](const auto& a,const auto& b){return a.score>b.score;});
  if(modes.empty())return {};
  auto best=modes.front();double second=0;
  for(const auto& mode:modes)
    if(std::hypot(mode.x-best.x,mode.y-best.y)*resolution>.35||std::abs(wrap(mode.yaw-best.yaw))>.35)second=std::max(second,mode.score);
  best.gap=best.score-second;best.unique=best.score>=.60&&best.gap>=.035;
  // Reject seeds with substantial occlusion: the measured ray cannot pass
  // through a mapped wall on its way to its endpoint.
  int blocked=0,total=0;const double co=std::cos(best.yaw),si=std::sin(best.yaw);
  for(const auto& p:points) {
    const double range=std::hypot(p.x,p.y);bool hit=false;
    for(double along=.25;along<range-.20;along+=.15) {
      const double factor=along/range;
      const int x=std::floor(best.x+(co*p.x-si*p.y)*factor/resolution);
      const int y=std::floor(best.y+(si*p.x+co*p.y)*factor/resolution);
      if(x>=0&&y>=0&&x<width&&y<height&&cells[y*width+x]>=65){hit=true;break;}
    }
    ++total;if(hit)++blocked;
  }
  if(total&&double(blocked)/total>.15)best.unique=false;
  return best;
}
}
