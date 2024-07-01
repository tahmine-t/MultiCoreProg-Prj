/* Sequential version of N body simulation */

#include "Dataset/NBody-600.h"
#include "VectorMath.h"
#include "CycleTimer.h"
#include <iostream>
#include <fstream>
#include <cuda.h>

using namespace std;

int COMPUTATION_STEP = 100;


// Compute forces on each body with time step

// Physics

__device__
void updateAcceleration(int bodyIndex, 
        Position3D *d_pos,
        Acceleration3D *d_acc,
        Mass *d_mass) 
{
   
  Force3D netForce = { 0, 0, 0 };

  for( int i = 0; i < BODY_COUNT; i++ ) 
  {
    if( i == bodyIndex ) 
    {
      continue;
    }

    Force3D vectorForceToOther = {0, 0, 0};
    Force scalarForceBetween = forceNewtonianGravity3D(
                                  d_mass[bodyIndex],
                                  d_mass[i],
                                  d_pos[bodyIndex],
                                  d_pos[i]);
    direction( 
      d_pos[bodyIndex],
      d_pos[i],
      vectorForceToOther);

    vectorForceToOther.x *= scalarForceBetween;
    vectorForceToOther.y *= scalarForceBetween;
    vectorForceToOther.z *= scalarForceBetween;
    netForce.x += vectorForceToOther.x;
    netForce.y += vectorForceToOther.y;
    netForce.z += vectorForceToOther.z;
  }

  d_acc[bodyIndex] = computeAccel3D(d_mass[bodyIndex], netForce);
}

__device__
void updateVelocity(
      int bodyIndex, 
      float deltaT, 
      Acceleration3D *d_acc,
      Velocity3D *d_vel
      ) 
{
  d_vel[bodyIndex] = computeVelo3D(
                                d_acc[bodyIndex],
                                d_vel[bodyIndex],
                                deltaT);
}

__device__
void updatePosition(
        int bodyIndex, 
        float deltaT,
        Velocity3D *d_vel,
        Position3D *d_pos) 
{

  d_pos[bodyIndex] = computePos3D( 
                              d_vel[bodyIndex],
                              d_pos[bodyIndex],
                              deltaT);
}

__global__
void updatePhysics(
        int bodies,
        float deltaT, 
        Position3D *d_pos,
        Velocity3D *d_vel,
        Acceleration3D *d_acc,
        Mass *d_mass)
{
  
  int i = blockIdx.x;
  int j = threadIdx.x;

  int body_id = (i * j) + j;

  if(body_id > bodies)
    return;
  
  updateAcceleration(body_id, d_pos, d_acc, d_mass);
  updateVelocity(body_id, deltaT, d_acc, d_vel);
  updatePosition(body_id, deltaT, d_vel, d_pos);
}

void display_bodies(ofstream &outfile)
{
  for (int i = 0; i < BODY_COUNT; i++){

    outfile << "Body " << i + 1 << ":" << endl;
    outfile << "Mass: " << nBodyMass[i] << endl;
    outfile << "Position(x, y, z): "
            << nBodyPosition[i].x << " " << nBodyPosition[i].y << " " << nBodyPosition[i].z << endl;
    outfile << "Velocity(x, y, z): "
            << nBodyVelocity[i].x << " " << nBodyVelocity[i].y << " " << nBodyVelocity[i].z << endl;
    outfile << "Acceleration(x, y, z): "
            << nBodyAcceleration[i].x << " " << nBodyAcceleration[i].y << " " << nBodyAcceleration[i].z << endl
            << endl;
  }
}


void compute(){

  int BYTES_SIZE_VECTOR = BODY_COUNT * sizeof(Vector3D);
  int BYTES_SIZE_SCALAR = BODY_COUNT * sizeof(Scalar);

  //Initializing Velocities of N bodies in GPU
  Velocity3D *h_vel = nBodyVelocity;
  Velocity3D *d_vel;
  cudaMalloc((void**) &d_vel, BYTES_SIZE_VECTOR);
  cudaMemcpy(d_vel, h_vel, BYTES_SIZE_VECTOR, cudaMemcpyHostToDevice);

  //Initializing acceleration of N bodies in GPU
  Acceleration3D *h_acc = nBodyAcceleration;
  Acceleration3D *d_acc;
  cudaMalloc((void**) &d_acc, BYTES_SIZE_VECTOR);
  cudaMemcpy(d_acc, h_acc, BYTES_SIZE_VECTOR, cudaMemcpyHostToDevice);
  
  //Initializing Mass of N bodies in GPU
  Mass *h_mass = nBodyMass;
  Mass *d_mass;
  cudaMalloc((void**) &d_mass, BYTES_SIZE_SCALAR);
  cudaMemcpy(d_mass, h_mass, BYTES_SIZE_SCALAR, cudaMemcpyHostToDevice);

  //Initializing Positions of N bodies in GPU
  Position3D *h_pos = nBodyPosition;
  Position3D *d_pos;
  cudaMalloc((void**) &d_pos, BYTES_SIZE_VECTOR);
  cudaMemcpy(d_pos, h_pos, BYTES_SIZE_VECTOR, cudaMemcpyHostToDevice);


  for (int i = 0; i < COMPUTATION_STEP; ++i){
    updatePhysics<<<(BODY_COUNT/16) + 1, 16>>>(BODY_COUNT, (float)(i * 100), d_pos, d_vel, d_acc, d_mass);
  }
    

  cudaMemcpy(h_pos, d_pos, BYTES_SIZE_VECTOR, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_vel, d_vel, BYTES_SIZE_VECTOR, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_acc, d_acc, BYTES_SIZE_VECTOR, cudaMemcpyDeviceToHost);
  cudaFree(d_pos);
  cudaFree(d_vel);
  cudaFree(d_acc);
  cudaFree(d_mass);

}


int main(){
  double start, end, min = 1e30;
  start = CycleTimer::currentSeconds();

  compute();
  
  end = CycleTimer::currentSeconds();
  min = std::min(min, end - start);

  // Write Results in output file
  ofstream file_name("nbody_parallel.txt");
  if (!file_name.is_open())
  {
    cerr << "Error opening file!" << endl;
    return 0;
  }
  file_name << "Body Count: " << BODY_COUNT << endl;
  file_name << "Total Time: " << min << " seconds" << endl
            << endl;
  display_bodies(file_name);
  file_name.close();
  
  printf("Time: %f\n", min);
}