// Native, isolated renderer regression. No robot, ROS, or shared scene.
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/math/Matrix4.hh>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
int main(int argc, char **argv) {
  if (argc != 6) return 2;
  const double fx=std::stod(argv[1]), fy=std::stod(argv[2]);
  const double cx=std::stod(argv[3]), cy=std::stod(argv[4]), hfov=std::stod(argv[5]);
  auto engine=ignition::rendering::engine("ogre2", {{"headless", "1"}});
  if (!engine) return 2;
  auto scene=engine->CreateScene("placement_prior_depth_plane_audit");
  auto camera=scene->CreateDepthCamera("depth_probe");
  camera->SetImageWidth(640); camera->SetImageHeight(360);
  camera->SetNearClipPlane(.26); camera->SetFarClipPlane(10.);
  camera->SetAspectRatio(640./360.); camera->SetHFOV(hfov);
  scene->RootVisual()->AddChild(camera);
  auto wall=scene->CreateVisual("slanted_plane");
  wall->AddGeometry(scene->CreateBox());wall->SetLocalScale(.01,10.,10.);
  wall->SetLocalPose(ignition::math::Pose3d(2.,0.,0.,0.,.2,.3));
  scene->RootVisual()->AddChild(wall);
  std::vector<float> values;
  auto connection=camera->ConnectNewDepthFrame([&](const float *data,unsigned int w,unsigned int h,unsigned int channels,const std::string &format){
    values.assign(data,data+w*h*channels);
  });
  camera->Update(); camera->Update();
  if (values.size()!=640*360) return 3;
  auto initial=values;
  auto requested=camera->ProjectionMatrix();
  requested(0,0)=2.*fx/640.; requested(1,1)=2.*fy/360.;
  requested(0,2)=1.-2.*cx/640.; requested(1,2)=2.*cy/360.-1.;
  camera->SetProjectionMatrix(requested);
  camera->Update();
  double difference=0.;
  for (size_t i=0;i<initial.size();++i)
    if(std::isfinite(initial[i])&&std::isfinite(values[i])) difference=std::max(difference,std::abs(double(initial[i]-values[i])));
  std::cout<<std::setprecision(17)<<"IMAGE_MAX_CHANGE="<<difference<<"\n";
  // Analytic front face of the box: n dot p = 2*n.x - half-thickness.
  // Camera coordinates are +X forward, +Y left, +Z up.
  const double nx=std::cos(.3)*std::cos(.2);
  const double ny=std::sin(.3)*std::cos(.2), nz=-std::sin(.2);
  double maxError=0.; int count=0;
  for (int v: {64,180,296}) for (int u:{64,320,576}) {
    double measured=values[v*640+u];
    double expected=(2.*nx-.005)/(nx-ny*(u-cx)/fx-nz*(v-cy)/fy);
    if (!std::isfinite(measured)) return 5;
    maxError=std::max(maxError,std::abs(measured-expected)); ++count;
    std::cout<<"PIXEL "<<u<<" "<<v<<" "<<measured<<" "<<expected<<"\n";
  }
  std::cout<<"MAX_DEPTH_ERROR_M="<<maxError<<" SAMPLE_COUNT="<<count<<"\n";
  // 0.1 mm is well above GPU numerical error and below depth quantization.
  bool pass=count==9 && maxError<.0001;
  connection.reset();camera.reset();wall.reset();
  engine->DestroyScene(scene);scene.reset();ignition::rendering::unloadEngine("ogre2");
  return pass?0:4;
}
