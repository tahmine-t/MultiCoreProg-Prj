# MultiCoreProg-Prj
Implementation of the N-body problem and its parallelization techniques through OpenMP and CUDA is.

## Overview
The N-body problem involves simulating the interactions between multiple celestial objects under gravitational forces, such as the motion of planets or stars. It becomes computationally intensive as the number of bodies increases, with the brute-force method yielding an \( O(n^2) \) complexity. 

### Approaches to Solve the N-Body Problem
1. **Brute Force (All Pairs)**:
   - Calculates gravitational forces between each pair of bodies.
   - Most accurate but computationally expensive with \( O(n^2) \) complexity.
   
2. **Barnes-Hut Algorithm**:
   - Approximation technique that uses a hierarchical spatial tree (octree) to group bodies that are far apart and reduces the number of calculations.
   - More efficient with \( O(n \log n) \) complexity.

## Serial Implementation
- The serial implementation calculates forces between all pairs, updates velocities, and computes new positions over several time steps. This method works well for small numbers of bodies but becomes impractical for large numbers due to its quadratic complexity.

## Parallel OpenMP Implementation
- OpenMP is used to distribute the computation of forces and body updates across multiple processors. This approach reduces computation time significantly, but the benefits level off with more than 16 threads due to overhead, resource contention, and synchronization issues.
- A **dynamic schedule** is used for varying workload across threads, and different setups, including thread binding, are tested.

## Parallel CUDA Implementation
- CUDA leverages GPU parallelism to handle large numbers of bodies by utilizing thousands of threads. Each thread computes the force, velocity, and position of a single body. The grid and block structure optimizes the workload distribution, while memory management and coalesced access patterns are crucial for performance.
- However, due to the all-to-all interaction between bodies, traditional tiling techniques aren't effective, as each body requires data from all others.
  
## Experimental Results
- **Serial Implementation**: The computation time increases significantly as the number of bodies grows.
- **OpenMP Implementation**: As the number of threads increases, computation time decreases, but after 16 threads, adding more threads can increase the time due to thread management overhead and resource contention. The best speedup from serial was over 4.5 times faster with 16 threads.
- **CUDA Implementation**: Performance improvement is noticeable, especially with smaller block sizes. However, for larger numbers of bodies, the optimal block size tends to change based on the workload, with results showing varying times for different block sizes.

## Running the code

To run the Serial version:
```
$ cd serial/
$ g++ -o nbody_serial n_body.cpp -lm
$ ./nbody_serial
```

To run the openMP version

Pre-requisites: CUDA, openMP and openGL libraries should be present
```
$ cd openMp/
$ g++ -o nbody_omp n_body_omp.cpp -fopenmp
$ ./nbody_omp
```

To run the CUDA version
```
$ nvcc -o nbody_cuda n_body_cuda.cu
$ ./nbody_cuda
```

To run the simulation
```
$ cd opengl-demo-nbody
$ make
$ ./n-body-sim
```
