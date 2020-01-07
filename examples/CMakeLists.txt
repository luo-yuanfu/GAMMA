set (CMAKE_CXX_STANDARD 11)

include_directories(${GAMMA_SOURCE_DIR}/src)

add_executable(predictor prediction/AgentInfo.cpp prediction/LearnBounds.cpp prediction/Predict.cpp prediction/WorldBelief.cpp)
target_link_libraries(predictor GAMMA)
add_test(predictor predictor)


