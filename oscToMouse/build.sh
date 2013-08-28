#!/bin/bash

gcc main.c -g3 -O0 --std=c99 \
`pkg-config --cflags --libs libmapper-0` \
-lxdo \
-o oscToMouse
