#pragma once
// Controller derivative extrema in normalized time [0,1]; no task margin.
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace luggage_planning {
namespace trajectory_bounds {
using Polynomial = std::vector<double>;  // ascending powers
inline double value(const Polynomial &p, double u) {
  double out = 0.;
  for (auto i=p.rbegin(); i!=p.rend(); ++i) out=out*u+*i;
  return out;
}
inline Polynomial derivative(const Polynomial &p) {
  Polynomial out;
  for (size_t i=1; i<p.size(); ++i) out.push_back(i*p[i]);
  return out;
}
// Derivative roots partition a polynomial into monotonic intervals. Isolate
// sign changes with binary64 bisection, retaining exact stationary roots too.
inline std::vector<double> roots(Polynomial p) {
  while (!p.empty() && p.back()==0.) p.pop_back();
  if (p.size()<2) return {};
  std::vector<double> boundary{0.};
  auto stationary=roots(derivative(p));
  boundary.insert(boundary.end(),stationary.begin(),stationary.end());
  boundary.push_back(1.);std::sort(boundary.begin(),boundary.end());
  std::vector<double> out;
  for (double u:boundary) if (value(p,u)==0.) out.push_back(u);
  for (size_t i=1; i<boundary.size(); ++i) {
    double lo=boundary[i-1],hi=boundary[i];
    double flo=value(p,lo),fhi=value(p,hi);
    if (flo==0. || fhi==0. || std::signbit(flo)==std::signbit(fhi)) continue;
    for (int step=0; step<64; ++step) {  // binary64 root isolation, not physical tolerance
      double mid=(lo+hi)/2.,fm=value(p,mid);
      if (fm==0.) {lo=hi=mid;break;}
      if (std::signbit(fm)==std::signbit(flo)) {lo=mid;flo=fm;} else hi=mid;
    }
    out.push_back((lo+hi)/2.);
  }
  std::sort(out.begin(),out.end());return out;
}
inline double maximum_absolute(const Polynomial &p) {
  double out=std::max(std::abs(value(p,0.)),std::abs(value(p,1.)));
  for (double u:roots(derivative(p))) out=std::max(out,std::abs(value(p,u)));
  if (!std::isfinite(out)) throw std::invalid_argument("nonfinite interpolation bound");
  return out;
}
inline std::pair<double,double> interval(double dt,double q0,double q1,
    double v0,double v1,double a0,double a1,int order) {
  if (!std::isfinite(dt) || dt<=0.) throw std::invalid_argument("positive interval required");
  for (double v:{q0,q1,v0,v1,a0,a1})
    if (!std::isfinite(v)) throw std::invalid_argument("finite interpolation values required");
  double delta=q1-q0;Polynomial position{q0,delta};
  if (order==3) {
    double vl=v0*dt,vr=v1*dt;
    position={q0,vl,3.*delta-2.*vl-vr,-2.*delta+vl+vr};
  } else if (order==5) {
    double vl=v0*dt,vr=v1*dt,al=a0*dt*dt,ar=a1*dt*dt;
    position={q0,vl,al/2.,10.*delta-6.*vl-4.*vr-1.5*al+.5*ar,
        -15.*delta+8.*vl+7.*vr+1.5*al-ar,6.*delta-3.*vl-3.*vr-.5*al+.5*ar};
  } else if (order!=1) throw std::invalid_argument("linear/cubic/quintic required");
  auto velocity=derivative(position);for (double &v:velocity) v/=dt;
  auto acceleration=derivative(velocity);for (double &v:acceleration) v/=dt;
  return {maximum_absolute(velocity),maximum_absolute(acceleration)};
}
}  // namespace trajectory_bounds
}  // namespace luggage_planning
