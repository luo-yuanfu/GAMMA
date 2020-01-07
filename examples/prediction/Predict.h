/*
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#ifndef GAMMA_PREDICTOR_H_
#define GAMMA_PREDICTOR_H_

#include "AgentInfo.h"
#include "LearnBounds.h"
#include <math.h>

using namespace std;
using namespace RVO;

class GammaPredictor{
public:

	vector <AgentInfo> agents_info_;
	ofstream output_file_;
	vector <Vector2> goals_;
	vector <Vector2> directions_;
	RVO::RVOSimulator* gamma_sim_;
	LearnBounds learn_bounds_;

	GammaPredictor ();
	vector<string> GetAllFiles(const char *path, string pattern);
	void SetObstacles(string agent_folder);
	void SetDirections(string agent_folder);
	void LoadData();
	void SetRoughDir();
	Vector2 PrefVel(AgentInfo agt, Vector2 cur_pos, int history_end_frame, Vector2 goal);
	void PreProcess();
	void PrintInfo();
	vector<Vector2> GetBoundingBoxCorners(AgentInfo agt, int frame_num, float ref_to_front, float ref_to_side, float ref_to_back);
	Vector2 ComputeGoalPositions(AgentInfo agt, int frame, int goal_id);
	void PredictAtOneFrame(int predict_begin_frame);
	Vector2 PredictOneStepForOneAgentAtOneFrame(int predict_begin_frame, int history_end_frame, AgentInfo agt, Vector2 goal);
	Vector2 PredictOneStepForOneAgentAtOneFrame(int predict_begin_frame, int history_end_frame, AgentInfo agt, Vector2 goal, float r_front);
	Vector2 PredictOneStepForOneAgentAtOneFrame(int predict_begin_frame, int history_end_frame, AgentInfo agt, Vector2 goal, float r_front, float res_dec_rate);
	void GammaPredict();
	vector<Vector2> BicycleMove (Vector2 cur_pos, Vector2 target_pos, Vector2 cur_heading, float max_speed, float car_len, float max_tracking_angle);

	inline double euclidean_dist(double x1, double y1, double x2, double y2){
		return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	}

	void run();
};

#endif
