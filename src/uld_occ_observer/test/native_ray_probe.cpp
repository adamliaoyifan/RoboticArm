#include "independent_oracle.hpp"
#include <octomap/OcTree.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

// Convention probe: independent DDA versus deployed liboctomap computeRayKeys.
// Disagreements are recorded; they are not used to generate O1 expected keys.

int main(int argc, char** argv) {
  const double res = 0.01;
  octomap::OcTree tree(res);
  std::vector<std::pair<observer_oracle::Vec3, observer_oracle::Vec3>> cases;
  cases.push_back({{0.005, 0.005, 0.005}, {0.205, 0.005, 0.005}});
  cases.push_back({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  cases.push_back({{0.001, 0.001, 0.001}, {1.001, 0.251, 0.501}});
  cases.push_back({{0.5, 0.5, 0.5}, {0.5 - 1e-12, 0.5, 0.5}});
  cases.push_back({{1.234, 0.01, 0.02}, {0.02, 1.5, 0.9}});
  cases.push_back({{-0.405, 0.985, 0.67}, {1.49, 0.2, 0.8}});
  cases.push_back({{-0.405, 0.985, 0.67}, {0.21, -0.4, 1.4}});
  cases.push_back({{2.7, 1.1, 0.4}, {-0.2, 0.3, 1.9}});
  int disagree = 0;
  std::ofstream out(argc > 1 ? argv[1] : "/dev/stdout");
  out << "{\"cases\":[\n";
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto& a = cases[i].first;
    const auto& b = cases[i].second;
    octomap::KeyRay ray;
    const bool ok = tree.computeRayKeys(octomap::point3d(a.x, a.y, a.z),
                                        octomap::point3d(b.x, b.y, b.z), ray);
    const auto independent = observer_oracle::traverse_ray(a, b, res);
    std::vector<observer_oracle::Cell> native;
    if (ok) {
      for (const auto& key : ray)
        native.push_back({static_cast<int>(key[0]) - 32768, static_cast<int>(key[1]) - 32768,
                          static_cast<int>(key[2]) - 32768});
    }
    const bool match = ok && native == independent;
    if (!match) ++disagree;
    out << " {\"i\":" << i << ",\"ok\":" << (ok ? "true" : "false") << ",\"match\":"
        << (match ? "true" : "false") << ",\"native\":" << native.size()
        << ",\"independent\":" << independent.size() << "}" << (i + 1 < cases.size() ? "," : "")
        << "\n";
  }
  out << "],\"disagreements\":" << disagree << "}\n";
  return 0;
}
