#!/bin/bash

cmake .
make
./trajectory_controller_2_codegen
cd trajectory_controller_mpc_solver_2
make

