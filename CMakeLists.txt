cmake_minimum_required(VERSION 3.5)
project(quadcopter_ompl_planner)

add_compile_options(-std=c++14)

# find_package(ODE REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  # ${ODE_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library (${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} PRIVATE src/quadcopter_ompl.cpp)

target_link_libraries(${PROJECT_NAME} 
    ${OMPL_LIBRARIES} 
    # ${ODE_LIBRARIES}
    ${EIGEN3_LIBRARIES})

# target_include_directories(
    # ${PROJECT_NAME}
    # PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
           # $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-config
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    EXPORT ${PROJECT_NAME}-config
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
