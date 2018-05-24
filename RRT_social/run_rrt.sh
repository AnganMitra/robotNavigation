#!/bin/bash

python rrt_star_reeds_shepp.py
rm *.pyc
mv *.png ./results/
