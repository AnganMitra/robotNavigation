#!/bin/bash

python simulate.py
rm *.pyc
mv *.png ./results/
mv *.gif ./results/
