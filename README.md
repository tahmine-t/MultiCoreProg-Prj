# MultiCoreProg-Prj

Running the code
To run the Serial version:
$ cd serial/
$ g++ -o nbody_serial n_body.cpp -lm
$ ./nbody_serial

To run the openMP version:
Pre-requisites: CUDA, openMP and openGL libraries should be present
$ cd openMp/
$ g++ -o nbody_omp n_body_omp.cpp -fopenmp
$ ./nbody_omp

To run the CUDA version
$ nvcc -o nbody_cuda n_body_cuda.cu
$ ./nbody_cuda

To run the simulation
$ cd opengl-demo-nbody
$ make
$ ./n-body-sim
