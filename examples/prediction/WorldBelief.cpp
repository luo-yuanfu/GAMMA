/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#include "WorldBelief.h"

default_random_engine generator;
uniform_real_distribution<float> uniform_dist(0.0f,1.0f);

AgentBelief::AgentBelief(){
}
AgentBelief::AgentBelief(int _id, Vector2 _pos, Vector2 _vel, vector <float> _prob_goals, vector <Vector2> _goal_positions){
	id = _id;
	pos = _pos;
	vel = _vel;
	prob_goals = _prob_goals;
	goal_positions = _goal_positions;
}
AgentBelief::AgentBelief(int _id, Vector2 _pos, Vector2 _vel, vector <float> _prob_goals, vector <Vector2> _goal_positions, vector <float> _prob_r_front, vector <float> _r_front_list, vector<float> _prob_combined_setting){
	id = _id;
	pos = _pos;
	vel = _vel;
	prob_goals = _prob_goals;
	goal_positions = _goal_positions;
	r_front_list = _r_front_list;
	prob_r_front = _prob_r_front;
	prob_combined_setting = _prob_combined_setting;
}
AgentBelief::AgentBelief(int _id, Vector2 _pos, Vector2 _vel, vector <float> _prob_goals, vector <Vector2> _goal_positions, vector <float> _prob_r_front, vector <float> _r_front_list, vector <float> _prob_res_dec_rate, vector <float> _res_dec_rate_list, vector<float> _prob_combined_setting){
	id = _id;
	pos = _pos;
	vel = _vel;
	prob_goals = _prob_goals;
	goal_positions = _goal_positions;
	r_front_list = _r_front_list;
	prob_r_front = _prob_r_front;
	prob_res_dec_rate = _prob_res_dec_rate;
	res_dec_rate_list = _res_dec_rate_list;
	prob_combined_setting = _prob_combined_setting;
}
int AgentBelief::sample_goal() {
	float r = uniform_dist(generator);
	int i = 0;
	r -= prob_goals[i];
	while(r > 0) {
		i++;
		r -= prob_goals[i];
	}
	return i;
}
int AgentBelief::maxlikely_goal() {
	double ml = 0;
	int mi = 0;
	for (size_t i=0; i<prob_goals.size(); i++) {
		if (prob_goals[i] > ml) {
			ml = prob_goals[i];
			mi = i;
		}
	}
	return mi;
}
int AgentBelief::sample_r_front() {
	float r = uniform_dist(generator);
	int i = 0;
	r -= prob_r_front[i];
	while(r > 0) {
		i++;
		r -= prob_r_front[i];
	}
	return i;
}
int AgentBelief::maxlikely_r_front() {
	double ml = 0;
	int mi = 0;
	for (size_t i=0; i<prob_r_front.size(); i++) {
		if (prob_r_front[i] > ml) {
			ml = prob_r_front[i];
			mi = i;
		}
	}
	return mi;
}
int AgentBelief::sample_r_rear() {
	float r = uniform_dist(generator);
	int i = 0;
	r -= prob_r_rear[i];
	while(r > 0) {
		i++;
		r -= prob_r_rear[i];
	}
	return i;
}
int AgentBelief::maxlikely_r_rear() {
	double ml = 0;
	int mi = 0;
	for (size_t i=0; i<prob_r_front.size(); i++) {
		if (prob_r_rear[i] > ml) {
			ml = prob_r_rear[i];
			mi = i;
		}
	}
	return mi;
}
int AgentBelief::sample_setting() {
	float r = uniform_dist(generator);
	int i = 0;
	r -= prob_combined_setting[i];
	while(r > 0) {
		i++;
		r -= prob_combined_setting[i];
	}
	return i;
}
int AgentBelief::maxlikely_prob_combined_setting() {
	double ml = 0;
	int mi = 0;
	for (size_t i=0; i<prob_combined_setting.size(); i++) {
		if (prob_combined_setting[i] > ml) {
			ml = prob_combined_setting[i];
			mi = i;
		}
	}
	return mi;
}


WorldBelief::WorldBelief (GammaPredictor *_predictor){
	predictor_ = _predictor;
}
vector<Vector2> WorldBelief::ComputeGoalPositions(AgentInfo agt, int frame){

	float time_of_acceleration = 0.0f;
	float time_of_uniform_speed = 0.0f;
	float total_time = (GlobalParams::predition_horizon) * GlobalParams::TIME_PER_FRAME;

	Vector2 cur_vel = agt.cur_vel [frame];
	Vector2 cur_acc = agt.cur_acc [frame];

	float max_speed = AgentParams::getDefaultAgentParam(agt.agent_type).maxSpeed;;

	float cur_speed = RVO::abs(cur_vel);
	if (cur_speed > max_speed) {
		cur_speed = max_speed;
		cur_vel = cur_speed * RVO::normalize (cur_vel);
	}
	float cur_acc_value = RVO::abs (cur_acc);

	Vector2 final_vel = cur_vel + total_time * cur_acc;
	if (final_vel * cur_vel < 0.0f) { // assume that agents can decelerate, but won't go in the opposite direction
		final_vel = Vector2 (0.0f,0.0f);
	}
	float final_speed = RVO::abs (final_vel);
	if (final_speed > max_speed) {
		final_speed = max_speed;
		final_vel = final_speed * RVO::normalize (final_vel);
	}

	if (cur_acc_value != 0.0f)
		time_of_acceleration = static_cast<float>( fabs(static_cast<double>(final_speed-cur_speed)) ) / cur_acc_value;
	else
		time_of_acceleration = 0.0f;
	if (time_of_acceleration > total_time)
		time_of_acceleration = total_time;
	time_of_uniform_speed = total_time - time_of_acceleration;
	Vector2 const_acc_pos = agt.pos[frame] + time_of_acceleration*0.5f*(cur_vel + final_vel) + time_of_uniform_speed * final_vel;
	Vector2 const_vel_pos = agt.pos [frame] + total_time * cur_vel;

	vector<Vector2> goal_positions;

	goal_positions.push_back (const_acc_pos);
	goal_positions.push_back (const_vel_pos);

	return goal_positions; 

}

AgentBelief WorldBelief::InitAgentBelief(AgentInfo agt, int frame) {

	vector <Vector2> goal_positions;

	goal_positions = ComputeGoalPositions (agt, frame - GlobalParams::history_size);

	vector <float> prob_goals;

	if (GlobalParams::dataset == DatasetNs::Cross_ct) {
		float prob = 1.0f / goal_positions.size();
		for (size_t i = 0; i < goal_positions.size(); i++) {
			prob_goals.push_back (prob);
		}
	} else if (GlobalParams::dataset == DatasetNs::Utown){
		if (agt.agent_type == "People") {
			prob_goals.push_back (0.2f);
			prob_goals.push_back (0.8f);
		} else {
			float prob = 1.0f / goal_positions.size();
			for (size_t i = 0; i < goal_positions.size(); i++) {
				prob_goals.push_back (prob);
			}
		}
	} else if (GlobalParams::dataset == DatasetNs::Eth || GlobalParams::dataset == DatasetNs::Hotel || GlobalParams::dataset == DatasetNs::Zara01 || 
		GlobalParams::dataset == DatasetNs::Zara02 || GlobalParams::dataset == DatasetNs::Univ001 || GlobalParams::dataset == DatasetNs::Univ003){
		prob_goals.push_back (0.2f);
		prob_goals.push_back (0.8f);

	} else{
		float prob = 1.0f / goal_positions.size();
		for (size_t i = 0; i < goal_positions.size(); i++) {
			prob_goals.push_back (prob);
		}
	}

	vector <float> prob_r_front;
	prob_r_front.push_back (0.7f);
	prob_r_front.push_back (0.3f);

	vector <float> prob_res_dec_rate;
	prob_res_dec_rate.push_back (0.7f);
	prob_res_dec_rate.push_back (0.3f);

	vector <float> prob_combined_setting;
	for (size_t i=0; i<prob_goals.size(); i++){
		for (size_t j = 0; j < prob_r_front.size(); j++) {
			for (size_t k = 0; k < prob_res_dec_rate.size(); k++) {
				prob_combined_setting.push_back (prob_goals [i] * prob_r_front [j] * prob_res_dec_rate[k]);
			}
		}
	}
	AgentBelief b;
	if (agt.agent_type == "People") {
		b = AgentBelief(agt.agent_id, agt.pos[frame], agt.cur_vel[frame], prob_goals, goal_positions, prob_r_front, GlobalParams::ped_r_front_list, prob_res_dec_rate, GlobalParams::ped_res_dec_rate_list, prob_combined_setting);
	} else if(agt.agent_type == "Car" || agt.agent_type == "Van" || agt.agent_type == "Bus" || agt.agent_type == "Jeep"){
		b = AgentBelief(agt.agent_id, agt.pos[frame], agt.cur_vel[frame], prob_goals, goal_positions, prob_r_front, GlobalParams::veh_r_front_list, prob_res_dec_rate, GlobalParams::veh_res_dec_rate_list, prob_combined_setting);
	} else{
		b = AgentBelief(agt.agent_id, agt.pos[frame], agt.cur_vel[frame], prob_goals, goal_positions, prob_r_front, GlobalParams::veh_r_front_list, prob_res_dec_rate, GlobalParams::bicycle_res_dec_rate_list, prob_combined_setting);
	}

	return b;

}

float WorldBelief::gaussian_prob(float x, float stddev) {
	float a = 1.0f / stddev / 2.506628f; //2.50662825325 = static_cast<float> Math.Sqrt(2 * 3.141593);
	float b = - x * x / 2.0f / (stddev * stddev);
	return a * static_cast<float> (exp(static_cast<float>(b)));
}

float WorldBelief::AgentMoveProb(Vector2 prev_pos, Vector2 cur_pos, Vector2 goal_pos, int predict_begin_frame, int history_end_frame, AgentInfo agt){
	float K = 0.0001f; // to avoid the extremely low probability
	Vector2 predicted_cur_pos = predictor_->PredictOneStepForOneAgentAtOneFrame (predict_begin_frame, history_end_frame, agt, goal_pos);
	float dist = RVO::abs (predicted_cur_pos - cur_pos);

	return gaussian_prob(dist, GlobalParams::dist_noise_std) + K;
}

float WorldBelief::AgentMoveProb(Vector2 prev_pos, Vector2 cur_pos, Vector2 goal_pos, int predict_begin_frame, int history_end_frame, float r_front, AgentInfo agt){
	float K = 0.0001f; // to avoid the extremely low probability
	Vector2 predicted_cur_pos = predictor_->PredictOneStepForOneAgentAtOneFrame (predict_begin_frame, history_end_frame, agt, goal_pos, r_front);
	float dist = RVO::abs (predicted_cur_pos - cur_pos);

	return gaussian_prob(dist, GlobalParams::dist_noise_std) + K;
}

float WorldBelief::AgentMoveProb(Vector2 prev_pos, Vector2 cur_pos, Vector2 goal_pos, int predict_begin_frame, int history_end_frame, float r_front, float res_dec_rate, AgentInfo agt){
	float K = 0.0001f; // to avoid the extremely low probability
	Vector2 predicted_cur_pos = predictor_->PredictOneStepForOneAgentAtOneFrame (predict_begin_frame, history_end_frame, agt, goal_pos, r_front, res_dec_rate);
	float dist = RVO::abs (predicted_cur_pos - cur_pos);

	return gaussian_prob(dist, GlobalParams::dist_noise_std) + K;
}

AgentBelief WorldBelief::ComputeAgentBelief(AgentInfo agt, int current_frame) {
	float SMOOTHING=0.0005f;

	AgentBelief b = InitAgentBelief (agt, current_frame);
	int history_begin_frame = current_frame - GlobalParams::history_size;

	// infer goals, r_front, and res_dec_rate
	b.pos = agt.pos[history_begin_frame];

	for (size_t frame = history_begin_frame + 1; frame <= current_frame; frame++) {
		float total_weight = 0;

		for (size_t i = 0; i < b.goal_positions.size(); i++) {
			for (int j = 0; j < b.r_front_list.size(); j++) {
				for (int k = 0; k < b.res_dec_rate_list.size(); k++) {
					int one_d_idx = i * b.r_front_list.size() * b.res_dec_rate_list.size() + j * b.res_dec_rate_list.size() + k;
					float prob = AgentMoveProb (b.pos, agt.pos [frame], b.goal_positions [i], frame - 1, current_frame, b.r_front_list[j], b.res_dec_rate_list[k], agt);
					b.prob_combined_setting [one_d_idx] *= prob;
					b.prob_combined_setting [one_d_idx] += SMOOTHING / b.prob_combined_setting.size();

					total_weight += b.prob_combined_setting [one_d_idx];
				}

			}
		}

		// normalize
		for (size_t i = 0; i < b.prob_combined_setting.size(); i++) {
			b.prob_combined_setting [i] /= total_weight;
		}

		b.pos = agt.pos [frame];

	}

	return b;

}

