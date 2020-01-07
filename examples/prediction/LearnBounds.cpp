/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#include "LearnBounds.h"

LearnBounds::LearnBounds (){

	speed_res_ = 0.5f;
	angle_res_ = 10.0f;
	max_speed_ = 10.0f;
	max_angle_ = 30.0f;

	speed_size = static_cast<size_t> (max_speed_ / speed_res_);
	angle_size = static_cast<size_t> (max_angle_ / angle_res_);
}

float LearnBounds::PurepusuitBicycleMoveError (float veh_speed, float traj_speed, float angle_diff, float moving_time, float max_speed, float car_len, float max_tracking_angle) {

	Vector2 bicycle_pos = Vector2 (0.0f, 0.0f);
	Vector2 bicycle_heading = Vector2 (0.0f, 1.0f);

	Vector2 traj_pos = Vector2 (0.0f, 0.0f);
	Vector2 traj_heading = bicycle_heading.rotate(angle_diff);

	float pursuit_time = 3.0f;
	float control_freq = 5.0f;
	float dt = 1.0f / control_freq;
	size_t simu_step = static_cast<size_t> (moving_time / dt);

	float max_error = 0.0f;
	float tmp_error;

	veh_speed = traj_speed;
	if (veh_speed > max_speed)
		veh_speed = max_speed;
	if (veh_speed < -max_speed)
		veh_speed = -max_speed;

	max_tracking_angle = max_tracking_angle * 3.1415f / 180.0f;
	float pursuit_len = 1.5f;
	if (car_len >= 2.5f && car_len <= 3.2f)
		pursuit_len = 2.0f;
	else if (car_len > 3.2f)
		pursuit_len = 3.0f;

	for (size_t i = 0; i < simu_step; i++) {
		Vector2 pursuit_pos = traj_pos + pursuit_len * traj_heading;
		float steer_ = (car_len/2.0f)*RVO::getSignedAngleRadOfTwoVector(pursuit_pos - bicycle_pos, bicycle_heading);
		if (steer_ > max_tracking_angle)
			steer_ = max_tracking_angle;
		if (steer_ < -max_tracking_angle)
			steer_ = -max_tracking_angle;
		float distance = veh_speed * dt;
		float turn = (float) std::tan (steer_) * distance / car_len;

		if ( (float) std::fabs (turn) < 0.0001f) { // use straight line model
			bicycle_pos += veh_speed * dt * bicycle_heading;
			bicycle_heading = bicycle_heading.rotate (turn * 180.0f / 3.14159f);

			traj_pos += traj_speed * dt * traj_heading;
			tmp_error = RVO::abs (bicycle_pos - traj_pos);
			if (tmp_error > max_error) {
				max_error = tmp_error;
			}

		}else { // use bicycle model
			float yaw =  (float) std::atan2((double)bicycle_heading.y(), (double)bicycle_heading.x());
			float radius = distance / turn;

			float cx = bicycle_pos.x() -  (float) std::sin ((double)yaw) * radius;
			float cy = bicycle_pos.y() +  (float) std::cos ((double)yaw) * radius;
			yaw = static_cast<float>(fmod(yaw + turn, 2 * 3.1415926f));
			float new_x = cx +  (float) std::sin ((double)yaw) * radius;
			float new_y = cy -  (float) std::cos ((double)yaw) * radius;

			bicycle_pos = Vector2 (new_x, new_y);
			bicycle_heading = bicycle_heading.rotate (turn * 180.0f / 3.14159f);

			traj_pos += traj_speed * dt * traj_heading;
			tmp_error = RVO::abs (bicycle_pos - traj_pos);
			if (tmp_error > max_error) {
				max_error = tmp_error;
			}
		}

	}

	return max_error;
}



void LearnBounds::ComputeSteerBoundTable() {


	vector<string> agent_types { "Car", "Scooter", "Van", "Bus", "Bicycle", "Electric_Tricycle", "Jeep" };

	float moving_time = GlobalParams::TIME_PER_FRAME;
	string agent_type;
	float agent_len;
	float error_bound;

	for (size_t k = 0; k < agent_types.size(); k++) {
		agent_type = agent_types[k];
		agent_len = AgentParams::getDefaultAgentParam(agent_type).len_rear_axle_to_front_axle;
		error_bound = AgentParams::getDefaultAgentParam(agent_type).error_bound;
		vector<float> steer_bounds;
		steer_bounds.resize(speed_size);
		for (size_t i = 0; i < speed_size; i++) {
			for (size_t j = angle_size - 1; j >= 0 ; j--) {
				float error = PurepusuitBicycleMoveError (i * speed_res_ + speed_res_ / 2.0f, i * speed_res_ + speed_res_ / 2.0f, j * angle_res_ + angle_res_ / 2.0f, moving_time, max_speed_, agent_len, max_angle_);
				if (error <= error_bound) {
					steer_bounds [i] = j * angle_res_ + angle_res_ / 2.0f;
					break;
				}
			}
		}
		steer_bound_table_dic[agent_type]=steer_bounds;
	}
}



void LearnBounds::ComputeVelocityConvexTable() {

	vector<string> agent_types { "Car", "Scooter", "Van", "Bus", "Bicycle", "Electric_Tricycle", "Jeep" };

	float moving_time = GlobalParams::TIME_PER_FRAME;
	string agent_type;
	float agent_len;
	float error_bound;

	Vector2 y_vel = Vector2(0.0f, 1.0f);

	for (size_t k = 0; k < agent_types.size(); k++) {
		agent_type = agent_types[k];
		agent_len = AgentParams::getDefaultAgentParam(agent_type).len_rear_axle_to_front_axle;
		error_bound = AgentParams::getDefaultAgentParam(agent_type).error_bound;
		vector<Vector2> boundary_vels;
		boundary_vels.resize(angle_size*2);
		for (size_t i = 0; i < angle_size; i++) {
			boundary_vels [0] = Vector2(0.0f, 0.0f);
			for (size_t j = speed_size - 1; j >= 0 ; j--) {
				float error = PurepusuitBicycleMoveError (j * speed_res_ + speed_res_ / 2.0f, j * speed_res_ + speed_res_ / 2.0f, 
					i * angle_res_, moving_time, max_speed_, agent_len, max_angle_);
				if (error <= error_bound) {
					if(i==0){
						boundary_vels [1] = (j * speed_res_ + speed_res_ / 2.0f) * y_vel;
					}else{
						boundary_vels [2*i] = (j * speed_res_ + speed_res_ / 2.0f) * y_vel.rotate(i * angle_res_);
						boundary_vels [2*i + 1] = (j * speed_res_ + speed_res_ / 2.0f) * y_vel.rotate(-i * angle_res_);
					}
					
					break;
				}
			}
		}
		vector<Vector2> tmp = RVO::makeConvexHull(boundary_vels);
		std::reverse(tmp.begin(), tmp.end());
		velocity_convex_table [agent_type] = tmp;
	}
}



