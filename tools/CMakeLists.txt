cmake_minimum_required(VERSION 3.16)
project(sid_tools C)

add_executable(sidtap2serial
	sidtap2serial.c
)
target_compile_features(sidtap2serial PRIVATE c_std_11)

add_executable(sid2serial
	sid2serial.c
)
target_compile_features(sid2serial PRIVATE c_std_11)
target_compile_definitions(sid2serial PRIVATE
	DEFAULT_VSID_PATH="${CMAKE_SOURCE_DIR}/vice-3.9/src/vsid"
	DEFAULT_FIFO_PATH="/tmp/sid.tap"
)
