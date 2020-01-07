/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#ifndef LEARN_BOUNDS_H_
#define LEARN_BOUNDS_H_

#include "AgentParams.h"
#include "AgentInfo.h"
#include <RVO.h>
#include <ConvexHull.h>
#include <vector>
#include <string>
#include <unordered_map>

using namespace RVO;
using namespace std;

class LearnBounds{
public:
	float speed_res_;
	float angle_res_;
	float max_speed_;
	float max_angle_;

	size_t speed_size;
	size_t angle_size;

	std::unordered_map <std::string, std::vector<float>> steer_bound_table_dic;
	std::unordered_map <std::string, std::vector<Vector2>> velocity_convex_table;
	

	LearnBounds ();

	float PurepusuitBicycleMoveError (float veh_speed, float traj_speed, float angle_diff, float moving_time, float max_speed, float car_len, float max_tracking_angle);

	void ComputeSteerBoundTable();

	void ComputeVelocityConvexTable();
};

#endif

