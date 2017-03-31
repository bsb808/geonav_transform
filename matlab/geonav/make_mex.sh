#!/bin/bash
mex -v CFLAGS='$CFLAGS -Wall' ll2xy.cpp
mex -v CFLAGS='$CFLAGS -Wall' xy2ll.cpp
