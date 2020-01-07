set (CMAKE_CXX_STANDARD 11)

set(GAMMA_HEADERS
	RVO.h
	RVOSimulator.h
	Vector2.h
	ConvexHull.h
	Minkowski.h
	GammaParams.h
	AgentParams.h)

set(GAMMA_SOURCES
	Agent.cpp
	Agent.h
	Definitions.h
	KdTree.cpp
	KdTree.h
	Obstacle.cpp
	Obstacle.h
	RVOSimulator.cpp
	ConvexHull.cpp
	GammaParams.cpp)

add_library(GAMMA ${GAMMA_HEADERS} ${GAMMA_SOURCES})

if(WIN32)
    set_target_properties(GAMMA PROPERTIES COMPILE_DEFINITIONS NOMINMAX)
endif()

install(FILES ${GAMMA_HEADERS} DESTINATION include)
install(TARGETS GAMMA DESTINATION lib)

