#!/bin/sh

# if [ $# -gt 0 ] ; then
#     base=`basename $1`
#     echo "compiling $base"
#     g++ -ggdb `pkg-config opencv --cflags --libs` $1 -o $base 
# if [ $# -gt 0 ] ; then
# 	base=`basename $1 .c`
# 	echo "compiling $base"
# 	gcc -ggdb `pkg-config opencv --cflags --libs` $base.c -o $base 
if [ $# -gt 0 ] ; then
    base=`basename $1 .cpp`
    echo "compiling $base"
    g++ -ggdb `pkg-config opencv --cflags --libs` -lglut -lGL -lGLU -lglut $base.cpp -o $base 
else
#	for i in *.c; do
#	    echo "compiling $i"
#	    gcc -ggdb `pkg-config --cflags opencv` -o `basename $i .c` $i `pkg-config --libs opencv` -lglut -lGL -lGLU -lglut -lm;
#	done
	for i in *.cpp; do
	    echo "compiling $i"
	    g++ -ggdb `pkg-config --cflags opencv` -o `basename $i .cpp` $i `pkg-config --libs opencv` -lglut -lGL -lGLU -lglut -lm;
	done
fi
