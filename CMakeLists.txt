cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
set (CMAKE_CXX_STANDARD 11)
find_package(MultiNEAT REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(spiderController SHARED src/RobotManager.cc
                                    src/CPG_NEAT_HYPERNEAT_Brain.cpp
                                    src/NeuralNetwork.cpp
                                    src/Oscillator.cpp
                                    src/JointPositionMotor.cpp
                                    src/Learner_NEAT_HyperNEAT.cpp
                                    src/utils.cpp)
target_link_libraries(spiderController ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} MultiNEAT::MultiNEAT)

add_library(worldController SHARED src/WorldController.cpp
                                    )
target_link_libraries(worldController ${GAZEBO_LIBRARIES})