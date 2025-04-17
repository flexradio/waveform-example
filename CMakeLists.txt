cmake_minimum_required(VERSION 3.16)
project(waveform-example C)
set(CMAKE_C_STANDARD 99)

find_package(Threads)
find_package(LibWaveform)

add_executable(waveform-example main.c)
target_link_libraries(waveform-example PRIVATE LibWaveform::waveform-static)
