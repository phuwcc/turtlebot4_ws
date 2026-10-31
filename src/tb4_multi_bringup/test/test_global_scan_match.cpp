#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../src/global_scan_match.hpp"
#include <cassert>
#include <deque>
#include <iostream>
static std::vector<int> distances(const std::vector<int8_t>& cells,int w) {
  std::vector<int>d(cells.size(),1000000);std::deque<int>q;
  for(size_t i=0;i<cells.size();++i)if(cells[i]>=65){d[i]=0;q.push_back(i);}
  while(!q.empty()){int i=q.front();q.pop_front();for(int j:{i%w?i-1:-1,i%w+1<w?i+1:-1,i>=w?i-w:-1,i+w<int(cells.size())?i+w:-1})if(j>=0&&d[j]>d[i]+1){d[j]=d[i]+1;q.push_back(j);}}
  return d;
}
static std::vector<tb4::ScanPoint> scan(const std::vector<int8_t>& cells,int w,double x,double y,double angle) {
  std::vector<tb4::ScanPoint> result;
  for(int i=0;i<72;++i){double a=i*2*M_PI/72;for(double r=.05;r<7;r+=.01){int ix=std::floor(x+std::cos(a+angle)*r/.05),iy=std::floor(y+std::sin(a+angle)*r/.05);if(ix<0||iy<0||ix>=w||iy>=w)break;if(cells[iy*w+ix]>=65){result.push_back({r*std::cos(a),r*std::sin(a)});break;}}}
  return result;
}
int main() {
  const int w=81;std::vector<int8_t> cells(w*w,0);
  for(int x=0;x<w;++x)for(int y=0;y<w;++y)if(x==0||y==0||x==w-1||y==w-1)cells[y*w+x]=100;
  auto symmetric=tb4::global_scan_match(w,w,.05,cells,distances(cells,w),scan(cells,w,40.5,40.5,0),3.);
  assert(!symmetric.unique);
  for(int x=19;x<25;++x)for(int y=28;y<38;++y)cells[y*w+x]=100;
  for(int x=52;x<61;++x)for(int y=51;y<55;++y)cells[y*w+x]=100;
  for(int x=10;x<45;++x)for(int y=60;y<63;++y)cells[y*w+x]=100;
  auto unique=tb4::global_scan_match(w,w,.05,cells,distances(cells,w),scan(cells,w,50.5,25.5,.3),3.);
  std::cout<<unique.x<<","<<unique.y<<" yaw="<<unique.yaw<<" score="<<unique.score<<" gap="<<unique.gap<<" unique="<<unique.unique<<"\n";
  assert(unique.unique);assert(std::hypot(unique.x-50.5,unique.y-25.5)<3);assert(std::abs(tb4::wrap(unique.yaw-.3))<.12);
  assert(!tb4::global_scan_match(w,w,.05,cells,distances(cells,w),scan(cells,w,50.5,25.5,.3),0.).unique);
}
