/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#include "AgentInfo.h"
#include "Predict.h"
#include "WorldBelief.h"
#include <dirent.h>
#include <sys/types.h>

using namespace std;
using namespace RVO;


GammaPredictor::GammaPredictor (){    
	output_file_.open("../gamma_output.txt");
	if (output_file_.fail()){
	  cout<<"cannot open file: ../gamma_output.txt"<<endl;
	}

	gamma_sim_ = new RVOSimulator();
}

vector<string> GammaPredictor::GetAllFiles(const char *path, string pattern) {
	struct dirent *entry;
	DIR *dir = opendir(path);
	vector<string> all_files;
	
	if (dir == NULL) {
	   return all_files;
	}
	while ((entry = readdir(dir)) != NULL) {

	   string filename = entry->d_name;
	   if (filename.find(pattern) != std::string::npos) {
	       all_files.push_back(entry->d_name);
	   }
	}
	closedir(dir);
	return all_files;
}

void GammaPredictor::LoadData(){

	string agent_folder = "../dataset/";
	string file_name_pattern = "frame";
	if (GlobalParams::dataset == DatasetNs::Cross_ct) {
		agent_folder += "Cross_ct/";
	} else if (GlobalParams::dataset == DatasetNs::Utown) {
		agent_folder += "UTown/";
	} else if (GlobalParams::dataset == DatasetNs::Eth) {
		agent_folder += "ETH/eth/";
	} else if (GlobalParams::dataset == DatasetNs::Hotel) {
		agent_folder += "ETH/hotel/";
	}else if (GlobalParams::dataset == DatasetNs::Zara01) {
		agent_folder += "UCY/zara1/";
	}else if (GlobalParams::dataset == DatasetNs::Zara02) {
		agent_folder += "UCY/zara2/";
	}else if (GlobalParams::dataset == DatasetNs::Univ001) {
		agent_folder += "UCY/univ/students001/";
	}else if (GlobalParams::dataset == DatasetNs::Univ003) {
		agent_folder += "UCY/univ/students003/";
	}

	vector<string> agent_files = GetAllFiles(agent_folder.c_str(), file_name_pattern);

	for (size_t i = 0; i < agent_files.size(); i++){
		string filename = agent_folder + agent_files[i];
		AgentInfo agt_info;
		agt_info.PreProcess (filename);
		agents_info_.push_back (agt_info);
	}

	SetObstacles(agent_folder);
	SetDirections(agent_folder);
}


void GammaPredictor::SetObstacles(string agent_folder){
	if (GlobalParams::dataset == DatasetNs::Cross_ct) {
		string obstacle_folder = agent_folder + "obstacles/";

		vector<string> obstacle_files = GetAllFiles(obstacle_folder.c_str(), "obstacle");

		for (size_t i = 0; i < obstacle_files.size(); i++){
			ifstream obst_file;
		    obst_file.open(obstacle_folder + obstacle_files[i], ifstream::in);

		    if(obst_file.fail()){
		        std::cout<<"cannot open obstacle file: "<<obstacle_files[i]<<std::endl;
		        return;
		    }

			vector <Vector2> obstacle;

			int id;
		    int time_frame;
		    float u;
		    float v;
		    float x;
		    float y;
		    string flag;
			string flag2;

			while (obst_file >>id >>time_frame >>u >>v >>x >>y >>flag >>flag2){
				obstacle.push_back(Vector2(x, y));
			}

			gamma_sim_->addObstacle(obstacle);

			obst_file.close();  
		}
		gamma_sim_->processObstacles();
	}
}

void GammaPredictor::SetDirections(string agent_folder){
	if (GlobalParams::dataset == DatasetNs::Cross_ct && GlobalParams::use_direction_prio) {
		string direction_folder = agent_folder + "directions/";

		vector<string> direction_files = GetAllFiles(direction_folder.c_str(), "direction");

		for (size_t i = 0; i < direction_files.size(); i++){
			ifstream dir_file;
		    dir_file.open(direction_folder + direction_files[i], ifstream::in);

		    if(dir_file.fail()){
		        std::cout<<"cannot open obstacle file: "<<direction_files[i]<<std::endl;
		        return;
		    }

			int id;
		    int time_frame;
		    float u;
		    float v;
		    float x;
		    float y;
		    string flag;
			string flag2;

			while (dir_file >>id >>time_frame >>u >>v >>x >>y >>flag >>flag2){
				Vector2 start_point = Vector2(x, y);
				if (dir_file >>id >>time_frame >>u >>v >>x >>y >>flag >>flag2) {
					Vector2 end_point = Vector2(x, y);
					Vector2 dir = end_point - start_point;
					dir = normalize (dir);
					directions_.push_back(dir);
					directions_.push_back(dir.rotate(180));
				}
			}

			dir_file.close();  
		}

		SetRoughDir ();
	}
}


void GammaPredictor::SetRoughDir(){
	for (size_t i = 0; i < agents_info_.size(); i++) {
		Vector2 end_pos = agents_info_ [i].pos [agents_info_ [i].end_time_frame];
		Vector2 start_pos = agents_info_ [i].pos [agents_info_ [i].start_time_frame];
		Vector2 dir = end_pos - start_pos;
		Vector2 closest_dir = dir;
		float min_angle = 10000.0f;
		int closest_dir_idx = 0;
		for (size_t j = 0; j < directions_.size(); j++ ) {
			float tmp = RVO::getSignedAngleRadOfTwoVector (dir, directions_ [j]);
			if (tmp < 0.0f) tmp = -tmp;
			if (tmp < min_angle) {
				min_angle = tmp;
				closest_dir = directions_ [j];
				closest_dir_idx = j;
			}
		}
		agents_info_ [i].rough_dir = closest_dir;
		agents_info_ [i].rough_dir_idx = closest_dir_idx;
	}

	vector <float> dir_count;
	dir_count.resize(directions_.size());
	for (size_t i = 0; i < directions_.size(); i++) {
		dir_count [i] = 0;
	}

	for (size_t i = 0; i < agents_info_.size(); i++) {
		if(agents_info_[i].end_time_frame - agents_info_[i].start_time_frame >= 15)
			dir_count [agents_info_ [i].rough_dir_idx]++;
	}
}


Vector2 GammaPredictor::PrefVel(AgentInfo agt, Vector2 cur_pos, int history_end_frame, Vector2 goal){

	float speed = agt.weighted_hist_speed [history_end_frame];

	Vector2 v_to_goal;

	v_to_goal = (goal - cur_pos) / GlobalParams::TIME_PER_FRAME;
	if(RVO::abs(v_to_goal) < speed)
		return v_to_goal;
	return speed * RVO::normalize (v_to_goal);
}


void GammaPredictor::PreProcess(){
	gamma_sim_->setTimeStep (GlobalParams::TIME_PER_FRAME);
	learn_bounds_.ComputeVelocityConvexTable ();
}

void GammaPredictor::PrintInfo(){
	for(size_t i = 0; i< agents_info_.size(); i++) {
		agents_info_[i].PrintAgentInfo ();
	}
	cout << "there are " << agents_info_.size() << "in total"  << endl;
}

vector<Vector2> GammaPredictor::GetBoundingBoxCorners(AgentInfo agt, int frame_num, float ref_to_front, float ref_to_side, float ref_to_back){

	Vector2 heading_rotate_90_clockwise = agt.cur_heading[frame_num].rotate(-90.0f);
	Vector2 front_right = agt.pos [frame_num] + ref_to_front * agt.cur_heading [frame_num] + ref_to_side * heading_rotate_90_clockwise;
	Vector2 front_left = agt.pos [frame_num] + ref_to_front * agt.cur_heading [frame_num] - ref_to_side * heading_rotate_90_clockwise;
	Vector2 back_right = agt.pos [frame_num] - ref_to_back * agt.cur_heading [frame_num] + ref_to_side * heading_rotate_90_clockwise;
	Vector2 back_left = agt.pos [frame_num] - ref_to_back * agt.cur_heading [frame_num] - ref_to_side * heading_rotate_90_clockwise;

	vector<Vector2> corners;
	corners.push_back(Vector2(back_left.x(), back_left.y()));
	corners.push_back(Vector2(back_right.x(), back_right.y()));
	corners.push_back(Vector2(front_right.x(), front_right.y()));
	corners.push_back(Vector2(front_left.x(), front_left.y()));

	return corners;
}



Vector2 GammaPredictor::ComputeGoalPositions(AgentInfo agt, int frame, int goal_id){
	
	float total_time = (GlobalParams::predition_horizon) * GlobalParams::TIME_PER_FRAME;
	Vector2 cur_vel = agt.cur_vel [frame];

	float max_speed = AgentParams::getDefaultAgentParam(agt.agent_type).maxSpeed;
	float cur_speed = RVO::abs(cur_vel);
	if (cur_speed > max_speed) {
		cur_speed = max_speed;
		cur_vel = cur_speed * RVO::normalize (cur_vel);
	}

	if(goal_id == 1) // const velocity model
		return agt.pos [frame] + total_time * cur_vel;


	float time_of_acceleration = 0.0f;
	float time_of_uniform_speed = 0.0f;
				
	Vector2 cur_acc = agt.cur_acc [frame];
	float cur_acc_value = RVO::abs (cur_acc);

	Vector2 final_vel = cur_vel + total_time * cur_acc;
	if (final_vel * cur_vel < 0.0f) { // assume that agents can decelerate, but won't go in the opposite direction
		final_vel = Vector2 (0.0f, 0.0f);
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
	
	// const acc model
	return agt.pos[frame] + time_of_acceleration*0.5f*(cur_vel + final_vel) + time_of_uniform_speed * final_vel;
}


//predict the future PREDICT_FRAME_NUM frames for each pedestrians from frame predict_begin_frame
void GammaPredictor::PredictAtOneFrame(int predict_begin_frame){

	cout<<"begin to predict at frame: "<<predict_begin_frame<<endl;

	GlobalParams::infer_goal = true;

	WorldBelief world(this);
	float delta_angle = 45.0f / GlobalParams::angle_d_size;
	vector<Vector2> goals;
	vector<float> r_front_list;
	vector<float> res_dec_rate_list;
	for (size_t i = 0; i < agents_info_.size(); i++) {
		
		if (agents_info_ [i].start_time_frame <= predict_begin_frame && agents_info_ [i].end_time_frame >= predict_begin_frame) {
	
			AgentBelief belief = world.ComputeAgentBelief (agents_info_[i], predict_begin_frame);

			int most_likely_setting_idx = belief.maxlikely_prob_combined_setting();
			int goal_id = most_likely_setting_idx /(GlobalParams::veh_r_front_list.size()*GlobalParams::veh_res_dec_rate_list.size());
			int r_front_idx = (most_likely_setting_idx % (GlobalParams::veh_r_front_list.size() * GlobalParams::veh_res_dec_rate_list.size())) / GlobalParams::veh_res_dec_rate_list.size();
			float r_front = belief.r_front_list [r_front_idx];
			r_front_list.push_back (r_front);
			int res_dec_rate_idx = (most_likely_setting_idx % (GlobalParams::veh_r_front_list.size() * GlobalParams::veh_res_dec_rate_list.size())) % GlobalParams::veh_res_dec_rate_list.size();
			float res_dec_rate = belief.res_dec_rate_list [res_dec_rate_idx];
			res_dec_rate_list.push_back (res_dec_rate);

			Vector2 predicted_goal = ComputeGoalPositions (agents_info_ [i], predict_begin_frame, goal_id); //goal_id
			float predicted_angle = (180.0f/3.1416f)*RVO::getSignedAngleRadOfTwoVector (predicted_goal - agents_info_[i].pos[predict_begin_frame], agents_info_[i].cur_heading[predict_begin_frame]);
			predicted_goal = agents_info_ [i].pos [predict_begin_frame] + (GlobalParams::prediction_time * RVO::abs (agents_info_ [i].cur_vel [predict_begin_frame])) * agents_info_[i].cur_heading[predict_begin_frame].rotate (predicted_angle);
			goals.push_back (predicted_goal);

		} else {
			goals.push_back (Vector2 (-1.0f, -1.0f));
			r_front_list.push_back (10.0f);
			res_dec_rate_list.push_back (0.15f);
		}
	}


	GlobalParams::infer_goal = false;
	gamma_sim_->clearAllAgents ();

	if(GlobalParams::record_results)
		output_file_<<"begin to predict at frame: "<<predict_begin_frame<<endl;

	int agent_num_in_frame = 0;
	for(size_t i=0; i<agents_info_.size(); i++){
		if(agents_info_[i].start_time_frame <= predict_begin_frame && agents_info_[i].end_time_frame >= predict_begin_frame){

			AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[i].agent_type);
			gamma_sim_->addAgent(default_agt, i);
			gamma_sim_->setAgentPosition(agent_num_in_frame, agents_info_[i].pos[predict_begin_frame]);
			gamma_sim_->setAgentVelocity(agent_num_in_frame, agents_info_[i].cur_vel[predict_begin_frame]);
			gamma_sim_->setAgentHeading (agent_num_in_frame, agents_info_[i].cur_heading[predict_begin_frame]);

			Vector2 pref_vel = PrefVel(agents_info_[i], agents_info_[i].pos[predict_begin_frame], predict_begin_frame, goals[i]);
			gamma_sim_->setAgentPrefVelocity(agent_num_in_frame, pref_vel);

			float len_ref_to_front = default_agt.len_ref_to_front;
			float len_ref_to_side = default_agt.len_ref_to_side;
			float len_ref_to_back = default_agt.len_ref_to_back;
			if(GlobalParams::use_enlarged_agents)
				len_ref_to_front += default_agt.error_bound / 2.0f;
			gamma_sim_->setAgentBoundingBoxCorners (agent_num_in_frame, GetBoundingBoxCorners(agents_info_[i], agent_num_in_frame, len_ref_to_front, len_ref_to_side, len_ref_to_back));

			if (GlobalParams::use_bounding_steer && agents_info_ [i].agent_type != "People") {
				gamma_sim_->setAgentVelocityConvex (agent_num_in_frame, learn_bounds_.velocity_convex_table[agents_info_ [i].agent_type]);
			}

			if (agents_info_ [i].agent_type == "People") {
				gamma_sim_->setAgentAttentionRadius (agent_num_in_frame, r_front_list[i], GlobalParams::ped_r_rear_list[0] );
				gamma_sim_->setAgentResDecRate (agent_num_in_frame, res_dec_rate_list[i] );
			} else if(agents_info_ [i].agent_type == "Car" || agents_info_ [i].agent_type == "Van" || agents_info_ [i].agent_type == "Bus"){
				gamma_sim_->setAgentAttentionRadius (agent_num_in_frame, r_front_list[i], GlobalParams::veh_r_rear_list[0] );
				gamma_sim_->setAgentResDecRate (agent_num_in_frame, res_dec_rate_list[i] );
			} else {
				gamma_sim_->setAgentAttentionRadius (agent_num_in_frame, r_front_list[i], GlobalParams::veh_r_rear_list[0] );
				gamma_sim_->setAgentResDecRate (agent_num_in_frame, res_dec_rate_list[i] );
			}

			agent_num_in_frame++;

		}
	}

	if(GlobalParams::record_results)
		output_file_<<"there are "<<agent_num_in_frame<<" agents in the frame."<<endl;

	if (agent_num_in_frame == 0)
		return;

	if(GlobalParams::record_results){
		for(size_t i=0;i<agent_num_in_frame; i++){
			int agent_id = gamma_sim_->getAgentID(i);
		
			output_file_<<agents_info_[agent_id].agent_id<<" "<<predict_begin_frame<<" "<<gamma_sim_->getAgentPosition(i).x()<<" "<<gamma_sim_->getAgentPosition(i).y()<<" ";
			output_file_<<agents_info_[agent_id].pos[predict_begin_frame].x()<<" "<<agents_info_[agent_id].pos[predict_begin_frame].y()<<endl;//ground truth position
		}
	}
	

	//predicted positions
	for(size_t j=predict_begin_frame+1; j<predict_begin_frame+GlobalParams::PREDICT_FRAME_NUM; j++){
		vector<Vector2> poses;
		vector<Vector2> headings;
		for (size_t i = 0; i < agent_num_in_frame; i++) {
			poses.push_back(gamma_sim_->getAgentPosition (i));
			headings.push_back(gamma_sim_->getAgentHeading (i));
		}

		gamma_sim_->doStep();

		for(size_t i=0;i<agent_num_in_frame; i++){

			AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[i].agent_type);

			int agent_id = gamma_sim_->getAgentID(i);
			float max_speed = default_agt.maxSpeed;

			if (GammaParams::consider_kinematics && agents_info_[agent_id].agent_type != "People") {

				float veh_len = default_agt.len_rear_axle_to_front_axle;
				float max_tracking_angle = default_agt.max_tracking_angle;

				vector<Vector2> pos_heading = BicycleMove (poses[i], gamma_sim_->getAgentPosition(i), headings[i], max_speed, veh_len, max_tracking_angle);
				if(GlobalParams::record_results) output_file_<<agents_info_[agent_id].agent_id<<" "<<j<<" "<<pos_heading[0].x()<<" "<<pos_heading[0].y()<<" ";
				gamma_sim_->setAgentPosition (i, pos_heading[0]);
				gamma_sim_->setAgentHeading (i, pos_heading[1]);
				gamma_sim_->setAgentPrefVelocity(i, PrefVel(agents_info_ [agent_id], pos_heading[0], predict_begin_frame, goals[agent_id]));

			} else {
				if(GlobalParams::record_results) output_file_<<agents_info_[agent_id].agent_id<<" "<<j<<" "<<gamma_sim_->getAgentPosition(i).x()<<" "<<gamma_sim_->getAgentPosition(i).y()<<" ";
				gamma_sim_->setAgentPrefVelocity(i, PrefVel(agents_info_ [agent_id],gamma_sim_->getAgentPosition(i), predict_begin_frame, goals[agent_id]));

			}

				
			if (j <= agents_info_ [agent_id].end_time_frame) {
				if(GlobalParams::record_results)
					output_file_<< agents_info_ [agent_id].pos [j].x () <<" "<< agents_info_ [agent_id].pos [j].y ()<<endl;//ground truth position

			} else {
				if(GlobalParams::record_results)
					output_file_<<"10000 10000"<<endl;
			}
			
		}
	}
}


Vector2 GammaPredictor::PredictOneStepForOneAgentAtOneFrame(int predict_begin_frame, int history_end_frame, AgentInfo agt, Vector2 goal){

	gamma_sim_->clearAllAgents ();

	int agent_num_in_frame = 0;
	int interested_agent_id_in_gamma = 0;
	Vector2 pos = Vector2(0.0f, 0.0f);
	Vector2 heading = Vector2(0.0f, 0.0f);
	for(size_t i=0; i<agents_info_.size(); i++){
		if(agents_info_[i].start_time_frame - GlobalParams::history_size <= predict_begin_frame && agents_info_[i].end_time_frame >= predict_begin_frame){
	
			AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[i].agent_type);
			gamma_sim_->addAgent(default_agt, i);
			gamma_sim_->setAgentPosition(agent_num_in_frame, agents_info_[i].pos[predict_begin_frame]);
			gamma_sim_->setAgentVelocity(agent_num_in_frame, agents_info_[i].cur_vel[predict_begin_frame]);
			gamma_sim_->setAgentHeading (agent_num_in_frame, agents_info_[i].cur_heading[predict_begin_frame]);

			float len_ref_to_front = default_agt.len_ref_to_front;
			float len_ref_to_side = default_agt.len_ref_to_side;
			float len_ref_to_back = default_agt.len_ref_to_back;
			if(GlobalParams::use_enlarged_agents)
				len_ref_to_front += default_agt.error_bound / 2.0f;
			gamma_sim_->setAgentBoundingBoxCorners (agent_num_in_frame, GetBoundingBoxCorners(agents_info_[i], agent_num_in_frame, len_ref_to_front, len_ref_to_side, len_ref_to_back));

			Vector2 pref_vel = Vector2 (0.0f, 0.0f);
			if (agents_info_ [i].agent_id == agt.agent_id) { // the agent of interest.
				pref_vel = PrefVel (agents_info_ [i], agents_info_ [i].pos[predict_begin_frame], history_end_frame - GlobalParams::history_size, goal);
				interested_agent_id_in_gamma = i; // note that both agents_info_ [i].agent_id and agt.agent_id are different from the agent_id we set in gamma. In gamma, we set i as the agent_id, while both agents_info_ [i].agent_id and agt.agent_id are values read from the dataset file
				pos = agents_info_[i].pos[predict_begin_frame];
				heading = agents_info_[i].cur_heading[predict_begin_frame];
			} else { // we dont know the goal of other agent when doing bayesian inference, hence we assume other agents will follow their current velocity
				pref_vel = agents_info_ [i].cur_vel[predict_begin_frame];
			}
			gamma_sim_->setAgentPrefVelocity (agent_num_in_frame, pref_vel);

			agent_num_in_frame++;
		}
	}


	if (agent_num_in_frame > 0)
		gamma_sim_->doStep(); // do one time step prediction

	for(size_t i=0;i<agent_num_in_frame; i++){

		int agent_id = gamma_sim_->getAgentID(i);

		if(agent_id != interested_agent_id_in_gamma){// we only consider the pose of the agent of interest
			continue;
		}

		AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[agent_id].agent_type);

		float max_speed = default_agt.maxSpeed;

		if (GammaParams::consider_kinematics && agents_info_[agent_id].agent_type != "People") {

			float veh_len = default_agt.len_rear_axle_to_front_axle;
			float max_tracking_angle = default_agt.max_tracking_angle;

			vector<Vector2> pos_heading = BicycleMove (pos, gamma_sim_->getAgentPosition(i), heading, max_speed, veh_len, max_tracking_angle);
			return pos_heading [0];
		} else {
			return gamma_sim_->getAgentPosition (i);
		}
	}

	return agt.pos [predict_begin_frame];
}



Vector2 GammaPredictor::PredictOneStepForOneAgentAtOneFrame(int predict_begin_frame, int history_end_frame, AgentInfo agt, Vector2 goal, float r_front){

	gamma_sim_->clearAllAgents ();

	int agent_num_in_frame = 0;
	int interested_agent_id_in_gamma = 0;
	Vector2 pos = Vector2(0.0f, 0.0f);
	Vector2 heading = Vector2(0.0f, 0.0f);

	for(size_t i=0; i<agents_info_.size(); i++){

		if(agents_info_[i].start_time_frame - GlobalParams::history_size <= predict_begin_frame && agents_info_[i].end_time_frame >= predict_begin_frame){

			AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[i].agent_type);
			gamma_sim_->addAgent(default_agt, i);
			gamma_sim_->setAgentPosition(agent_num_in_frame, agents_info_[i].pos[predict_begin_frame]);
			gamma_sim_->setAgentVelocity(agent_num_in_frame, agents_info_[i].cur_vel[predict_begin_frame]);
			gamma_sim_->setAgentHeading (agent_num_in_frame, agents_info_[i].cur_heading[predict_begin_frame]);

			float len_ref_to_front = default_agt.len_ref_to_front;
			float len_ref_to_side = default_agt.len_ref_to_side;
			float len_ref_to_back = default_agt.len_ref_to_back;
			if(GlobalParams::use_enlarged_agents)
				len_ref_to_front += default_agt.error_bound / 2.0f;
			gamma_sim_->setAgentBoundingBoxCorners (agent_num_in_frame, GetBoundingBoxCorners(agents_info_[i], agent_num_in_frame, len_ref_to_front, len_ref_to_side, len_ref_to_back));

			Vector2 pref_vel = Vector2 (0.0f, 0.0f);
			if (agents_info_ [i].agent_id == agt.agent_id) { // the agent of interest.
				pref_vel = PrefVel (agents_info_ [i], agents_info_ [i].pos[predict_begin_frame], history_end_frame - GlobalParams::history_size, goal);
				interested_agent_id_in_gamma = i; // note that both agents_info_ [i].agent_id and agt.agent_id are different from the agent_id we set in gamma. In gamma, we set i as the agent_id, while both agents_info_ [i].agent_id and agt.agent_id are values read from the dataset file
				pos = agents_info_[i].pos[predict_begin_frame];
				heading = agents_info_[i].cur_heading[predict_begin_frame];
			} else { // we dont know the goal of other agent when doing bayesian inference, hence we assume other agents will follow their current velocity
				pref_vel = agents_info_ [i].cur_vel[predict_begin_frame];
			}
			gamma_sim_->setAgentPrefVelocity (agent_num_in_frame, pref_vel);

			gamma_sim_->setAgentAttentionRadius (agent_num_in_frame, r_front, GlobalParams::ped_r_rear_list[0]);

			agent_num_in_frame++;
		}
	}

	if (agent_num_in_frame > 0)
		gamma_sim_->doStep(); // do one time step prediction

	for(size_t i=0;i<agent_num_in_frame; i++){

		int agent_id = gamma_sim_->getAgentID(i);

		if(agent_id != interested_agent_id_in_gamma){// we only consider the pose of the agent of interest
			continue;
		}

		AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[agent_id].agent_type);

		float max_speed = default_agt.maxSpeed;

		if (GammaParams::consider_kinematics && agents_info_[agent_id].agent_type != "People") {

			float veh_len = default_agt.len_rear_axle_to_front_axle;
			float max_tracking_angle = default_agt.max_tracking_angle;

			vector<Vector2> pos_heading = BicycleMove (pos, gamma_sim_->getAgentPosition(i), heading, max_speed, veh_len, max_tracking_angle);
			return pos_heading [0];
		} else {
			return gamma_sim_->getAgentPosition (i);
		}
	}

	return agt.pos [predict_begin_frame];
}



Vector2 GammaPredictor::PredictOneStepForOneAgentAtOneFrame(int predict_begin_frame, int history_end_frame, AgentInfo agt, Vector2 goal, float r_front, float res_dec_rate){


	gamma_sim_->clearAllAgents ();

	int agent_num_in_frame = 0;
	int interested_agent_id_in_gamma = 0;
	Vector2 pos = Vector2(0.0f, 0.0f);
	Vector2 heading = Vector2(0.0f, 0.0f);

	for(size_t i=0; i<agents_info_.size(); i++){

		if(agents_info_[i].start_time_frame - GlobalParams::history_size <= predict_begin_frame && agents_info_[i].end_time_frame >= predict_begin_frame){
	
			AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[i].agent_type);
			gamma_sim_->addAgent(default_agt, i);
			gamma_sim_->setAgentPosition(agent_num_in_frame, agents_info_[i].pos[predict_begin_frame]);
			gamma_sim_->setAgentVelocity(agent_num_in_frame, agents_info_[i].cur_vel[predict_begin_frame]);
			gamma_sim_->setAgentHeading (agent_num_in_frame, agents_info_[i].cur_heading[predict_begin_frame]);

			float len_ref_to_front = default_agt.len_ref_to_front;
			float len_ref_to_side = default_agt.len_ref_to_side;
			float len_ref_to_back = default_agt.len_ref_to_back;
			if(GlobalParams::use_enlarged_agents)
				len_ref_to_front += default_agt.error_bound / 2.0f;
			gamma_sim_->setAgentBoundingBoxCorners (agent_num_in_frame, GetBoundingBoxCorners(agents_info_[i], agent_num_in_frame, len_ref_to_front, len_ref_to_side, len_ref_to_back));

			Vector2 pref_vel = Vector2 (0.0f, 0.0f);
			if (agents_info_ [i].agent_id == agt.agent_id) { // the agent of interest.
				pref_vel = PrefVel (agents_info_ [i], agents_info_ [i].pos[predict_begin_frame], history_end_frame - GlobalParams::history_size, goal);
				interested_agent_id_in_gamma = i; // note that both agents_info_ [i].agent_id and agt.agent_id are different from the agent_id we set in gamma. In gamma, we set i as the agent_id, while both agents_info_ [i].agent_id and agt.agent_id are values read from the dataset file
				pos = agents_info_[i].pos[predict_begin_frame];
				heading = agents_info_[i].cur_heading[predict_begin_frame];
			} else { // we dont know the goal of other agent when doing bayesian inference, hence we assume other agents will follow their current velocity
				pref_vel = agents_info_ [i].cur_vel[predict_begin_frame];
			}
			gamma_sim_->setAgentPrefVelocity (agent_num_in_frame, pref_vel);
			gamma_sim_->setAgentAttentionRadius (agent_num_in_frame, r_front, GlobalParams::ped_r_rear_list[0]);
			gamma_sim_->setAgentResDecRate (agent_num_in_frame, res_dec_rate);

			agent_num_in_frame++;
		}
	}
	if (agent_num_in_frame > 0)
		gamma_sim_->doStep(); // do one time step prediction

	for(size_t i=0;i<agent_num_in_frame; i++){

		int agent_id = gamma_sim_->getAgentID(i);

		if(agent_id != interested_agent_id_in_gamma){//we only consider the pose of the agent of interest
			continue;
		}

		AgentParams default_agt = AgentParams::getDefaultAgentParam(agents_info_[agent_id].agent_type);

		float max_speed = default_agt.maxSpeed;

		if (GammaParams::consider_kinematics && agents_info_[agent_id].agent_type != "People") {

			float veh_len = default_agt.len_rear_axle_to_front_axle;
			float max_tracking_angle = default_agt.max_tracking_angle;

			vector<Vector2> pos_heading = BicycleMove (pos, gamma_sim_->getAgentPosition(i), heading, max_speed, veh_len, max_tracking_angle);
			return pos_heading [0];
		} else {
			return gamma_sim_->getAgentPosition (i);
		}
	}

	return agt.pos [predict_begin_frame];
}


void GammaPredictor::GammaPredict(){
	for(size_t i=1; i<GlobalParams::MAX_FRAME; i++){
		PredictAtOneFrame(i);
	}
}


vector<Vector2> GammaPredictor::BicycleMove (Vector2 cur_pos, Vector2 target_pos, Vector2 cur_heading, float max_speed, float car_len, float max_tracking_angle) {

	int simu_step = 3;
	vector<Vector2> pos_heading;
	float dt = GlobalParams::TIME_PER_FRAME / simu_step;
	Vector2 pref_vel = (target_pos - cur_pos) / GlobalParams::TIME_PER_FRAME;

	float speed_ = RVO::abs (pref_vel); //this is equivalent to making k0 infinite, where k0 is the control param for acceleration P controller: a = k0*(v* - v)
	if (speed_ == 0) {
		pos_heading.push_back (cur_pos);
		pos_heading.push_back (cur_heading);
		return pos_heading;
	}

	if (speed_ > max_speed)
		speed_ = max_speed;

	max_tracking_angle = max_tracking_angle * 3.1415f / 180.0f;


	for (size_t i = 0; i < simu_step; i++) {
		
		float steer_ = (car_len/6.8f)*RVO::getSignedAngleRadOfTwoVector(target_pos - cur_pos, cur_heading);
		if (steer_ > max_tracking_angle)
			steer_ = max_tracking_angle;
		if (steer_ < -max_tracking_angle)
			steer_ = -max_tracking_angle;

		float distance = speed_ * dt;

		float turn = (float) std::tan (steer_) * distance / car_len;

		if ( (float) fabs (turn) < 0.0001f) { // use straight line model
			cur_pos += (target_pos-cur_pos)/simu_step;
			cur_heading = cur_heading.rotate (turn * 180.0f / 3.14159f);
		}else { // use bicycle model
			float yaw =  (float) std::atan2((double)cur_heading.y(), (double)cur_heading.x());
			float radius = distance / turn;

			float cx = cur_pos.x() -  (float) std::sin ((double)yaw) * radius;
			float cy = cur_pos.y() +  (float) std::cos ((double)yaw) * radius;
			yaw = static_cast<float>(fmod(yaw + turn, 2 * 3.1415926f));

			float new_x = cx +  (float) std::sin ((double)yaw) * radius;
			float new_y = cy -  (float) std::cos ((double)yaw) * radius;

			cur_pos = Vector2 (new_x, new_y);
			cur_heading = cur_heading.rotate (turn * 180.0f / 3.14159f);
		}
	
	}

	pos_heading.push_back (cur_pos);
	pos_heading.push_back (cur_heading);
	return pos_heading;
}
	
void GammaPredictor::run(){

	LoadData ();
	//PrintInfo ();

	PreProcess ();

	GammaPredict ();

	output_file_.close ();
}

int main()
{
	GammaPredictor p;
	p.run();

	return 0;
}
