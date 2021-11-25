cmake_minimum_required(VERSION 3.5)
project(quadcopter_ompl_planner)

add_compile_options(-std=c++14)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} PRIVATE src/quadcopter_ompl.cpp)

target_link_libraries(${PROJECT_NAME} 
    ${OMPL_LIBRARIES} 
    ${EIGEN3_LIBRARIES})

add_executable(test_${PROJECT_NAME} test/test.cpp)

target_link_libraries(test_${PROJECT_NAME} 
    ${PROJECT_NAME})

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
