from __future__ import print_function
import numpy as np
import copy
import math
import glob

all_traj_total_gamma_ade = 0.0
all_traj_total_gamma_fde = 0.0

trajectory_count = 0.0

agent_id_list = []

M = [[55.8303947300378, -21.4969860492286, 1119.51180638000], [-38.7841290361716, 31.3801203371366, 694.848288076000], [0.0412819851646424, 0.0501909553699786, 1]]
M = np.array(M)

def transform_world_to_pixel(world_x_list, world_y_list):
    uv_list = []
    for i in range(0, len(world_x_list)):
        uv = np.dot(M, np.array([world_x_list[i],world_y_list[i], 1.0]))
        uv /= uv[2]
        uv_list.append(uv)
    return uv_list

def divide_trajectories(agent_id):

	global trajectory_segment
	global content
	global prediction_frame_count

	frame_start = -1
	frame_end = -1
	frame_tmp = 1
	
	for line in content:
		if "begin to predict at frame:" in line:
			frame_tmp = int(line.split(' ')[5])
		elif "there are" not in line:
			if int(line.split(' ')[0]) == agent_id:
				if frame_start == -1:
					frame_start = frame_tmp
				frame_end = frame_tmp

	
	if frame_start != -1 and frame_end != -1:
		while frame_start + prediction_frame_count <= frame_end:
			trajectory_segment[agent_id].append(frame_start)
			trajectory_segment[agent_id].append(frame_start + prediction_frame_count)
			frame_start += 1 #prediction_frame_count #1


def compute_accuracy(agent_trajectories, agent_id):
	traj_num = len(agent_trajectories) / 2

	global prediction_frame_count

	total_gamma_ade = 0.0
	total_gamma_fde = 0.0

	start_a_new_traj = False
	prediction_frame_count_itr = 0

	gamma_x_list = []
	gamma_y_list = []
	gamma_all_x_list = []
	gamma_all_y_list = []

	ori_x_list = []
	ori_y_list = []
	ori_all_x_list = []
	ori_all_y_list = []

	for line in content:
		if "begin to predict at frame:" in line and int(line.split(' ')[5]) > agent_trajectories[(traj_num-1)*2+1]:
			if gamma_x_list != []:
				gamma_all_x_list.append(gamma_x_list)
				gamma_all_y_list.append(gamma_y_list)
				ori_all_x_list.append(ori_x_list)
				ori_all_y_list.append(ori_y_list)
			gamma_x_list = []
			gamma_y_list = []
			ori_x_list = []
			ori_y_list = []
			break
		elif "begin to predict at frame:" in line and int(line.split(' ')[5]) >= agent_trajectories[0]:
			frame_tmp = int(line.split(' ')[5])

			for i in range(traj_num):
				if frame_tmp == agent_trajectories[i*2]:
					start_a_new_traj = True
					prediction_frame_count_itr = 0
					if gamma_x_list != []:
						gamma_all_x_list.append(gamma_x_list)
						gamma_all_y_list.append(gamma_y_list)
						gamma_x_list = []
						gamma_y_list = []
						ori_all_x_list.append(ori_x_list)
						ori_all_y_list.append(ori_y_list)
						ori_x_list = []
						ori_y_list = []
					break
				else:
					start_a_new_traj = False
					prediction_frame_count_itr = 0
			

		elif start_a_new_traj and "there are" not in line:
			if agent_id == int(line.split(' ')[0]) and prediction_frame_count_itr < prediction_frame_count:
				if prediction_frame_count_itr == prediction_frame_count - 1:
					total_gamma_fde += dist(float(line.split(' ')[2]), float(line.split(' ')[3]), float(line.split(' ')[4]), float(line.split(' ')[5]))
					
					gamma_x_list.append(float(line.split(' ')[2]))
					gamma_y_list.append(float(line.split(' ')[3]))
					ori_x_list.append(float(line.split(' ')[4]))
					ori_y_list.append(float(line.split(' ')[5]))
				
				total_gamma_ade += dist(float(line.split(' ')[2]), float(line.split(' ')[3]), float(line.split(' ')[4]), float(line.split(' ')[5]))
				gamma_x_list.append(float(line.split(' ')[2]))
				gamma_y_list.append(float(line.split(' ')[3]))
				ori_x_list.append(float(line.split(' ')[4]))
				ori_y_list.append(float(line.split(' ')[5]))

				prediction_frame_count_itr += 1

	if output_file_for_visualization:
		for i in range(len(gamma_all_x_list)):
			output_dir = "pos_in_image_space/"
			file_name = "frame_"+str(agent_trajectories[0])+"_agent_"+str(agent_id)+"_"+str(i)+"_gamma.txt"

			output_file = open(output_dir+file_name, "w")
			output_file2 = open(output_dir+file_name[:-9]+"ori.txt", "w")

			uv_list = transform_world_to_pixel(gamma_all_x_list[i], gamma_all_y_list[i])
			start_frame = agent_trajectories[2*i]
			for j in range(len(uv_list)):
				print(agent_id, start_frame+j, uv_list[j][0], uv_list[j][1], file = output_file)

			uv_list = transform_world_to_pixel(ori_all_x_list[i], ori_all_y_list[i])
			start_frame = agent_trajectories[2*i]
			for j in range(len(uv_list)):
				print(agent_id, start_frame+j, uv_list[j][0], uv_list[j][1], file = output_file2)

			output_file.close()
			output_file2.close()

	return traj_num, total_gamma_ade, total_gamma_fde


def dist(x1,y1,x2,y2):
	return math.sqrt((x1-x2)**2+(y1-y2)**2)

total_agent_num = 450
filename = "gamma_output.txt"
file_handler = open(filename,'r')
content = file_handler.read().splitlines()

prediction_duration = 4.8
prediction_frame_count = int(prediction_duration * 2.5)

trajectory_segment = [copy.deepcopy([]) for i in range(450)]

output_file_for_visualization = False

for i in range(0, total_agent_num):
	divide_trajectories(i)

for i in range(0, total_agent_num):
	if trajectory_segment[i] != []:
		traj_num, total_gamma_ade, total_gamma_fde = compute_accuracy(trajectory_segment[i], i)

		all_traj_total_gamma_ade += total_gamma_ade
		all_traj_total_gamma_fde += total_gamma_fde

		trajectory_count += float(traj_num)


print ("total number of trajectories: ", trajectory_count)
print ("ade: ", all_traj_total_gamma_ade / trajectory_count / prediction_frame_count)
print ("fde: ", all_traj_total_gamma_fde / trajectory_count)

print("finished")


