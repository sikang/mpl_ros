#include "timer.hpp"
#include "map_reader.hpp"
#include <collision_checking/voxel_map_util.h>
#include <planner/mp_map_util.h>

#define VISUALIZE 1
#if VISUALIZE
#include <vtkSmartPointer.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkJPEGWriter.h> 
#endif

using namespace MPL;

int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Load the map 
  MapReader<Vec3i, Vec3f> reader(argv[1]); 
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot find input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  // Pass the data into a VoxelMapUtil class for collision checking
  std::shared_ptr<VoxelMapUtil> map_util;
  map_util.reset(new VoxelMapUtil);
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  map_util->dilate(0.1, 0.1); // Dilate obstacles with r = 0.1, h = 0.1
  map_util->dilating(); // Dilating

  // Initialize planning mission 
  Waypoint start, goal;
  start.pos = Vec3f(2.5, -3.5, 0.0); 
  start.vel = Vec3f::Zero(); 
  start.acc = Vec3f::Zero(); 
  start.jrk = Vec3f::Zero(); 
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false; 
  start.use_jrk = false; 

  goal.pos = Vec3f(37, 2.5, 0.0);
  goal.vel = Vec3f::Zero(); 
  goal.acc = Vec3f::Zero(); 
  goal.jrk = Vec3f::Zero(); 
 
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  std::unique_ptr<MPMapUtil> planner(new MPMapUtil(true)); // Declare a mp planner using voxel map
  planner->setMapUtil(map_util); // Set collision checking function
  planner->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner->setVmax(1.0); // Set max velocity
  planner->setAmax(1.0); // Set max acceleration 
  planner->setJmax(1.0); // Set max jerk
  planner->setUmax(0.5); // Set max control input
  planner->setDt(1.0); // Set dt for each primitive
  planner->setW(10); // Set dt for each primitive
  planner->setMaxNum(-1); // Set maximum allowed states
  planner->setU(1, false);// 2D discretization with 1
  planner->setMode(start); // Set effort degree
  planner->setTol(1, 1, 1); // Tolerance for goal region

  // Planning
  Timer time(true);
  bool valid = planner->plan(start, goal); // Plan from start to goal
  double dt = time.Elapsed().count();
  printf("MP Planner takes: %f ms\n", dt);
  printf("MP Planner expanded states: %zu\n", planner->getCloseSet().size());

#if VISUALIZE
  // Plot the result in image
  const Vec3i dim = reader.dim();
  const Vec3i startI = map_util->floatToInt(start.pos);
  const Vec3i goalI= map_util->floatToInt(goal.pos);
  std::string outputFilename = "output.jpg"; 
  // Create a 100x100 image to save into the jpeg file
  int extent[6] = { 0, dim(0), 0, dim(1), 0, 0 };
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetExtent( extent );
  imageSource->SetScalarTypeToUnsignedChar(); // vtkJPEGWriter only accepts unsigned char input
  imageSource->SetNumberOfScalarComponents( 3 ); // 3 color channels: Red, Green and Blue

  // Fill the whole image with a white background
  imageSource->SetDrawColor( 255, 255, 255 );
  imageSource->FillBox( extent[0], extent[1], extent[2], extent[3] );

  imageSource->SetDrawColor(127.0, 127.0, 127.0);
  for(int x = 0; x < dim(0); x ++) {
    for(int y = 0; y < dim(1); y ++) {
      if(!map_util->isFree(Vec3i(x, y, 0))) {
        imageSource->DrawPoint(x, y);
      }
    }
  }

  imageSource->SetDrawColor(0.0, 127.0, 255.0);
  imageSource->DrawCircle(startI[0], startI[1], 5);

  imageSource->SetDrawColor(255.0, 0.0, 0.0);
  imageSource->DrawCircle(goalI[0], goalI[1], 5);

  //Plot expended states
  for(const auto& pt: planner->getCloseSet()) {
    imageSource->SetDrawColor(155.0, 155.0, 155.0);
    const Vec3i pi = map_util->floatToInt(pt);
    imageSource->DrawCircle(pi[0], pi[1], 2);
  }

  if(valid) {
    Trajectory traj = planner->getTraj();
    decimal_t total_t = traj.getTotalTime();
    printf("Total time T: %f\n", total_t);
    printf("Total J:  J(0) = %f, J(1) = %f, J(2) = %f, J(3) = %f\n", 
        traj.J(0), traj.J(1), traj.J(2), traj.J(3));
    int num = 1000; // number of points on trajectory to draw
    imageSource->SetDrawColor(0.0, 0.0, 0.0);
    for(int i = 0; i < num -1 ; i ++) {
      const decimal_t t1 = i * total_t / num; 
      const decimal_t t2 = (i + 1) * total_t / num; 
      Waypoint w1, w2;
      traj.evaluate(t1, w1);
      traj.evaluate(t2, w2);
      const Vec3i p1 = map_util->floatToInt(w1.pos);
      const Vec3i p2 = map_util->floatToInt(w2.pos);
      imageSource->FillTube(p1[0], p1[1], p2[0], p2[1], 2);
    }

  }

#endif


  vtkSmartPointer<vtkJPEGWriter> writer = vtkSmartPointer<vtkJPEGWriter>::New();  writer->SetFileName( outputFilename.c_str() );  writer->SetInputConnection( imageSource->GetOutputPort() );  writer->Write();

  return 0;
}
