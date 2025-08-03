cmake_minimum_required(VERSION 3.6)
project(testApp)
set(ntv_H
	"nvec3.h"
	"nmat3.h"
	"nmat4.h"
)
add_executable(testApp tester.C)
