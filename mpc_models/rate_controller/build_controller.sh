#!/bin/bash

cmake .
make
./rate_controller_2_codegen
cd rate_controller_mpc_solver_2
make

