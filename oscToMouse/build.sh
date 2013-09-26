#!/bin/bash

gcc main.c -g3 -O0 --std=c99 \
`pkg-config --cflags --libs libmapper-0` \
`pkg-config --cflags --libs opencv` \
`pkg-config --cflags --libs liblo` \
-lxdo \
-o oscToMouse
