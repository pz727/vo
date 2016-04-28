#!/bin/bash
./ate_evaluate.py --plot ATE groundtruth.txt traj.txt
./rpe_evaluate.py --fixed_delta --plot RPE --verbose groundtruth.txt traj.txt
