/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#include "AgentInfo.h"


namespace GlobalParams{
	const DatasetNs::Dataset dataset = DatasetNs::Eth;
	const int MAX_FRAME = (dataset == DatasetNs::Utown? 700: (dataset == DatasetNs::Cross_ct? 421:(dataset == DatasetNs::Eth? 2100:(dataset == DatasetNs::Hotel? 1825:(dataset == DatasetNs::Zara01? 900:(dataset == DatasetNs::Zara02? 1050:(dataset == DatasetNs::Univ001? 450:550)))))));
	const float  TIME_PER_FRAME = 0.4f;
	const int PREDICT_FRAME_NUM = 12;
	const bool use_predefined_goal = false;
	const bool use_front_goals = true;
	const bool use_direction_prio = true;
	const int angle_d_size = 9;
	const int history_size = 2;
	const int predition_horizon = 12;  // in number of frame
	const float prediction_time = 4.8;
	const float dist_noise_std = 0.25f;
	bool infer_goal = true;
	const float total_error = 0;
	const float total_count = 0;
	const float stop_speed = 0.15f;
	std::vector<float> ped_r_front_list{ 6.0f, 8.0f};
	std::vector<float> ped_r_rear_list{0.0f, 1.0f};
	std::vector<float> ped_res_dec_rate_list{ 0.6f, 0.8f };
	std::vector<float> veh_r_front_list{ 8.0f, 10.0f};
	std::vector<float> veh_r_rear_list{0.0f, 1.0f};
	std::vector<float> veh_res_dec_rate_list{ 0.1f, 0.2f };
	std::vector<float> bicycle_res_dec_rate_list{ 0.3f, 0.4f };
	const bool is_print_results = true;
	const bool use_enlarged_agents = true;
	const bool use_bounding_steer = true;
	const bool use_combined_speed = false;

	const bool record_results = true;
	const float discount = ( (dataset == DatasetNs::Eth || dataset == DatasetNs::Hotel)? 0.8f:0.0f);
};


AgentInfo::AgentInfo ()
{
	pos.resize(GlobalParams::MAX_FRAME);
	cur_vel.resize(GlobalParams::MAX_FRAME);
	cur_acc.resize(GlobalParams::MAX_FRAME);
	cur_heading.resize(GlobalParams::MAX_FRAME);
	weighted_hist_speed.resize(GlobalParams::MAX_FRAME);
}

AgentInfo::AgentInfo(int id){
	AgentInfo ();
	agent_id = id;
}

std::string AgentInfo::GetAgentType(std::string type){
	if (type == "moped" || type == "scooter" || type == "motorbike" || type == "motorcycle" || type == "Scooter")
		return "Scooter";
	else if (type == "pedestrian" || type == "people" || type == "People")
		return "People";
	else if (type == "car" || type == "Car")
		return "Car";
	else if (type == "van" || type == "Van")
		return "Van";
	else if (type == "bus" || type == "Bus")
		return "Bus";
	else if (type == "jeep" || type == "Jeep")
		return "Jeep";
	else if (type == "bicycle" || type == "Bicycle")
		return "Bicycle";
	else if (type == "electric_tricycle" || type == "Electric_Tricycle")
		return "Electric_Tricycle";
	else if (type == "gyro_scooter" || type == "Gyro_Scooter")
		return "Gyro_Scooter";
	else return "Car";
}

void AgentInfo::ReadData(std::string filename){

	std::ifstream file;

    file.open(filename, std::ifstream::in);

    if(file.fail()){
        std::cout<<"open "<<filename<<" failed"<<std::endl;
        return;
    }

	std::string line;
	int id;
	int time_frame = 0;
	float u;
	float v;
	float x;
	float y;
	float dx;
	float dy;
	std::string type;
	std::string ref_point;

	bool is_first_frame = true;

	if(GlobalParams::dataset == DatasetNs::Cross_ct || GlobalParams::dataset == DatasetNs::Utown){
		while(file >> id >> time_frame >> u >> v >> x >> y >> dx >> dy >> type >> ref_point) {  
			if(time_frame>=GlobalParams::MAX_FRAME-1){
				time_frame --;
				break;
			}
			if(is_first_frame){
				start_time_frame = time_frame;
				agent_id = id;
				agent_type = GetAgentType(type);
				ref_point = ref_point;
				is_first_frame = false;
			}

			pos[time_frame] = Vector2(x,y);

			if (GlobalParams::dataset == DatasetNs::Cross_ct) {
				cur_heading [time_frame] = normalize(Vector2(dx,dy));
			}

		}

	} else {
		while(file >>id >>time_frame >>u >>v >>x >>y) {  
			if(time_frame>=GlobalParams::MAX_FRAME-1){
				time_frame --;
				break;
			}
			if(is_first_frame){
				start_time_frame = time_frame;
				agent_id = id;
				agent_type = "People";
				ref_point = "rear_axle_center";
				is_first_frame = false;
			}

			pos[time_frame] = Vector2(x,y);
		}
	}
	 

	end_time_frame = time_frame;

	file.close();
}


void AgentInfo::ComputeCurVel(){
	if(end_time_frame - start_time_frame <= 1) return;
	Vector2 vel;

	for (int i=start_time_frame+1; i<=end_time_frame; i++){
		vel = Vector2((pos[i].x()-pos[i-1].x())/(GlobalParams::TIME_PER_FRAME), (pos[i].y()-pos[i-1].y())/(GlobalParams::TIME_PER_FRAME));
		float speed = abs (vel);
		cur_vel [i] = speed * cur_heading [i];

		float his_speed = 0.0f;
		float weight = 0;
		float discount = GlobalParams::discount;

		his_speed += speed;
		weight += 1.0;

		for(int j=1; j<=3; j++){
			if(i-j>=start_time_frame+1){
				his_speed += discount * RVO::abs (cur_vel [i-j]);
				weight += discount;
				discount = discount * discount;
			}
		}
		
		weighted_hist_speed[i] = his_speed / weight;
	}
	cur_vel[start_time_frame] = cur_vel[start_time_frame+1];
	weighted_hist_speed[start_time_frame] = weighted_hist_speed[start_time_frame+1];	
}
void AgentInfo::ComputeCurAcc(){
	if(end_time_frame - start_time_frame <= 1) return;
	for (int i=start_time_frame+2; i<=end_time_frame; i++){
		cur_acc[i] = (cur_vel[i]-cur_vel[i-1])/(GlobalParams::TIME_PER_FRAME);
	}
	cur_acc[start_time_frame] = cur_acc[start_time_frame+1] = cur_acc[start_time_frame+2];
}

void AgentInfo::ComputeHeading(){
	if(end_time_frame - start_time_frame < 1) return;
	for (int i=start_time_frame+1; i<=end_time_frame; i++){
		cur_heading[i] = normalize((pos[i]-pos[i-1]));
	}
	cur_heading[start_time_frame] = cur_heading[start_time_frame+1];
}

void AgentInfo::PreProcess(std::string filename){
	ReadData(filename);
	ComputeGoal();
	if (GlobalParams::dataset != DatasetNs::Cross_ct) {
		ComputeHeading ();
	}
	ComputeCurVel();
	ComputeCurAcc();

	start_time_frame += 4; // the first four frames are used as history
}
void AgentInfo::ComputeGoal(){
	goal = Vector2(pos[end_time_frame].x(), pos[end_time_frame].y());
}

void AgentInfo::PrintAgentInfo(){
}
