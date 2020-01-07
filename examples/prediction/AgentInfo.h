/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#ifndef AGENT_INFO_H_
#define AGENT_INFO_H_

#include <RVO.h>
#include <GammaParams.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "AgentParams.h"

namespace DatasetNs {
	enum Dataset{
		Cross_ct,
		Utown,
		Eth,
		Hotel,
		Zara01,
		Zara02,
		Univ001,
		Univ003
	};
};

namespace GlobalParams{
	extern const DatasetNs::Dataset dataset;
	extern const int MAX_FRAME;
	extern const float  TIME_PER_FRAME;
	extern const int PREDICT_FRAME_NUM;
	extern const bool use_predefined_goal;
	extern const bool use_front_goals;
	extern const bool use_direction_prio;
	extern const int angle_d_size;
	extern const int history_size;
	extern const int predition_horizon;
	extern const float prediction_time;
	extern const float dist_noise_std;
	extern bool infer_goal;
	extern const float total_error;
	extern const float total_count;
	extern const float stop_speed;
	extern std::vector<float> ped_r_front_list;
	extern std::vector<float> ped_r_rear_list;
	extern std::vector<float> ped_res_dec_rate_list;
	extern std::vector<float> veh_r_front_list;
	extern std::vector<float> veh_r_rear_list;
	extern std::vector<float> veh_res_dec_rate_list;
	extern std::vector<float> bicycle_res_dec_rate_list;
	extern const bool is_print_results;
	extern const bool use_enlarged_agents;
	extern const bool use_bounding_steer;
	extern const bool use_combined_speed;

	extern const bool record_results;
	extern const float discount;
};

using namespace RVO;
class AgentInfo
{
public:
	std::vector<Vector2> pos;
	std::vector<Vector2> cur_vel;
	std::vector<Vector2> cur_acc;
	std::vector<Vector2> cur_heading;
	std::vector<float> weighted_hist_speed;

	Vector2 goal;
	int agent_id;
	int start_time_frame;
	int end_time_frame;
	std::string agent_type;
	std::string ref_point;

	float history_ave_speed;

	Vector2 rough_dir;
	int rough_dir_idx;

	AgentInfo ();

	AgentInfo(int id);

	void ReadData(std::string filename);

	void ComputeCurVel();
	void ComputeCurAcc();

	void ComputeHeading();

	void PreProcess(std::string filename);
	void ComputeGoal();

	std::string GetAgentType(std::string type);
	void PrintAgentInfo();
};

#endif