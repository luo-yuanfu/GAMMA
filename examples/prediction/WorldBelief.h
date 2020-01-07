/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#ifndef WORLD_BELIEF_H_
#define WORLD_BELIEF_H_

#include "AgentParams.h"
#include "AgentInfo.h"
#include "Predict.h"
#include <RVO.h>
#include <vector>
#include <string>

#include <iostream>
#include <random>
#include <math.h>

using namespace RVO;
using namespace std;

class AgentBelief {
public:
	int id;
	Vector2 pos;
	Vector2 vel;

	vector <float> prob_goals;
	vector <Vector2> goal_positions;

	vector <float> prob_r_front;
	vector <float> r_front_list;

	vector <float> prob_r_rear;
	vector <float> r_rear_list;

	vector <float> prob_res_dec_rate;
	vector <float> res_dec_rate_list;

	vector <float> prob_combined_setting;

	AgentBelief();
	AgentBelief(int _id, Vector2 _pos, Vector2 _vel, vector <float> _prob_goals, vector <Vector2> _goal_positions);
	AgentBelief(int _id, Vector2 _pos, Vector2 _vel, vector <float> _prob_goals, vector <Vector2> _goal_positions, vector <float> _prob_r_front, vector <float> _r_front_list, vector<float> _prob_combined_setting);
	AgentBelief(int _id, Vector2 _pos, Vector2 _vel, vector <float> _prob_goals, vector <Vector2> _goal_positions, vector <float> _prob_r_front, vector <float> _r_front_list, vector <float> _prob_res_dec_rate, vector <float> _res_dec_rate_list, vector<float> _prob_combined_setting);
	int sample_goal();
	int maxlikely_goal();
	int sample_r_front();
	int maxlikely_r_front();
	int sample_r_rear();
	int maxlikely_r_rear();
	int sample_setting();
	int maxlikely_prob_combined_setting();
};

class WorldBelief{
public:
	GammaPredictor * predictor_;

	WorldBelief ();
	WorldBelief (GammaPredictor *_predictor);

	vector<Vector2> ComputeGoalPositions(AgentInfo agt, int frame);

	AgentBelief InitAgentBelief(AgentInfo agt, int frame);

	float gaussian_prob(float x, float stddev);

	float AgentMoveProb(Vector2 prev_pos, Vector2 cur_pos, Vector2 goal_pos, int predict_begin_frame, int history_end_frame, AgentInfo agt);

	float AgentMoveProb(Vector2 prev_pos, Vector2 cur_pos, Vector2 goal_pos, int predict_begin_frame, int history_end_frame, float r_front, AgentInfo agt);

	float AgentMoveProb(Vector2 prev_pos, Vector2 cur_pos, Vector2 goal_pos, int predict_begin_frame, int history_end_frame, float r_front, float res_dec_rate, AgentInfo agt);

	AgentBelief ComputeAgentBelief(AgentInfo agt, int current_frame);
};

#endif

