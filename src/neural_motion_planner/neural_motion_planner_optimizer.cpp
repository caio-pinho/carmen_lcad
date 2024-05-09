/*
 * neural_motion_planner_optimizer.cpp
 *
 *  Created on: Mar 22, 2023
 *      Author: caiopinho
 */


#include <stdio.h>
#include <iostream>
#include <math.h>

#include <carmen/collision_detection.h>
#include <carmen/carmen.h>

#include "robot_state.h"
#include "model/global_state.h"
#include "util.h"
#include "neural_motion_planner_optimizer.h"

#include <fstream>
#include <vector>
#include <cmath>
#include <limits>

//#define PUBLISH_PLAN_TREE
#ifdef PUBLISH_PLAN_TREE
#include "model/tree.h"
#include "publisher_util.h"

void
copy_path_to_traj(carmen_robot_and_trailer_traj_point_t *traj, vector<carmen_robot_and_trailer_path_point_t> path);
#endif

#define G_STEP_SIZE	0.001
#define F_STEP_SIZE	G_STEP_SIZE

#define G_TOL		0.01
#define F_TOL		G_TOL

#define G_EPSABS	0.016
#define F_EPSABS	G_EPSABS

bool use_obstacles = true;

double steering_previous = 0.0;

extern int use_unity_simulator;

template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); 
  return linspaced;
}


void print_vector(std::vector<double> vec)
{
  std::cout << "size: " << vec.size() << std::endl;
  for (double d : vec)
    std::cout << d << " ";
  std::cout << std::endl;
}


std::vector<double> get_predicted_vehicle_location(double x, double y, double steering_angle, double yaw, double v, double t) {
	double wheel_heading = yaw + steering_angle;
	double wheel_traveled_dis = v * t; //(timestamp - this->vars.t_previous);
	return {x + wheel_traveled_dis * cos(wheel_heading), y + wheel_traveled_dis * sin(wheel_heading)};
}


double get_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}


carmen_robot_and_trailer_path_point_t
convert_to_carmen_robot_and_trailer_path_point_t(const carmen_robot_and_trailer_traj_point_t robot_state, const double time)
{
	carmen_robot_and_trailer_path_point_t path_point;

	path_point.x = robot_state.x;
	path_point.y = robot_state.y;
	path_point.theta = robot_state.theta;
	path_point.beta = robot_state.beta;
	path_point.v = robot_state.v;
	path_point.phi = robot_state.phi;
	path_point.time = time;

	return (path_point);
}


double
compute_path_via_simulation(carmen_robot_and_trailer_traj_point_t &robot_state, Command &command,
		vector<carmen_robot_and_trailer_path_point_t> &path,
		TrajectoryControlParameters tcp,
		double v0, double i_beta, double delta_t)
{

	robot_state.x = 0.0;
	robot_state.y = 0.0;
	robot_state.theta = 0.0;
	robot_state.beta = i_beta;
	robot_state.v = v0;
	robot_state.phi = 0.0;

	command.v = v0;
	double multiple_delta_t = 3.0 * delta_t;
	int i = 0;
	double t;
	double distance_traveled = 0.0;
	path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, delta_t));
	
	std::vector<double> steering_list = linspace(-0.05235988, 0.05235988, 21);
	
	for (t = delta_t; t < tcp.tt; t += delta_t)
	{
		t = tcp.tt;
		command.v = v0 + tcp.a * t;
		command.v = GlobalState::robot_config.max_v;
		if (command.v > GlobalState::param_max_vel)
			command.v = GlobalState::param_max_vel;
		else if (command.v < GlobalState::param_max_vel_reverse)
			command.v = GlobalState::param_max_vel_reverse;
		
		for (auto& steering : steering_list) {
			steering += steering_previous;
		}

		double minimum_d = std::numeric_limits<double>::infinity();
		for (unsigned int i = 0; i < steering_list.size(); i++) {
    		std::vector<double> predicted_vehicle_location = get_predicted_vehicle_location(GlobalState::localizer_pose->x, GlobalState::localizer_pose->y, steering_list[i], GlobalState::localizer_pose->theta, command.v, t);
			double d_to_s1 = get_distance(predicted_vehicle_location[0], predicted_vehicle_location[1], GlobalState::goal_pose->x, GlobalState::goal_pose->y);
			if (d_to_s1 < minimum_d) { 
        		command.phi = steering_list[i]; 
        		minimum_d = d_to_s1;
    		}
		}
		steering_previous = command.phi;

		if ((GlobalState::semi_trailer_config.type != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN))
			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, delta_t,
					&distance_traveled, delta_t / 10.0, GlobalState::robot_config, GlobalState::semi_trailer_config);
		else
			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, t / 2,
					&distance_traveled, delta_t, GlobalState::robot_config, GlobalState::semi_trailer_config);

		path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, delta_t));
		
		if (GlobalState::eliminate_path_follower && (i > 70))
			delta_t = multiple_delta_t;
		i++;
	}
	
	return (distance_traveled);
}


double
get_max_distance_in_path(vector<carmen_robot_and_trailer_path_point_t> path, carmen_robot_and_trailer_path_point_t &furthest_point)
{
	double max_dist = 0.0;

	furthest_point = path[0];
	for (unsigned int i = 0; i < path.size(); i++)
	{
		double distance = DIST2D(path[0], path[i]);
		if (distance > max_dist)
		{
			max_dist = distance;
			furthest_point = path[i];
		}
	}

	return (max_dist);
}


vector<carmen_robot_and_trailer_path_point_t>
simulate_car_from_parameters(TrajectoryDimensions &td,
		TrajectoryControlParameters &tcp, double v0, double i_beta, double delta_t)
{
	vector<carmen_robot_and_trailer_path_point_t> path = {};
	if (!tcp.valid)
		return (path);

	Command command;
	carmen_robot_and_trailer_traj_point_t robot_state;
	double distance_traveled = compute_path_via_simulation(robot_state, command, path, tcp, v0, i_beta, delta_t);

	carmen_robot_and_trailer_path_point_t furthest_point;
	td.dist = get_max_distance_in_path(path, furthest_point);
	td.theta = atan2(furthest_point.y, furthest_point.x);
	td.d_yaw = furthest_point.theta;
	td.phi_i = 0.0;
	td.v_i = v0;
	tcp.vf = command.v;
	tcp.sf = distance_traveled;
	td.control_parameters = tcp;

	return (path);
}


bool
path_has_loop(double dist, double sf)
{
	if (dist < 0.05)
		return (false);

	if (sf > (M_PI * dist * 1.1)) // se sf for maior que meio arco com diametro dist mais um pouco (1.1) tem loop
		return (true);
	return (false);
}


bool
bad_tcp(TrajectoryControlParameters tcp)
{
	if (isnan(tcp.tt) || isnan(tcp.a) || isnan(tcp.s))
		return (true);
	else
		return (false);
}


void
compute_a_and_t_from_s_reverse(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * target_td.dist);
	a = (-1.0) * a;
	if (a == 0.0)
	{
		if (target_v != 0.0)
			tcp_seed.tt = fabs(s / target_v);
		else
			tcp_seed.tt = 0.05;
	}
	else if (a < -GlobalState::robot_config.maximum_acceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_acceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v) + v)) / a;
		a = (-1.0) * a;

	}
	else if (a > GlobalState::robot_config.maximum_deceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_deceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v)) + v) / a;
	}
	else
		tcp_seed.tt = (target_v - target_td.v_i) / a;

	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s_foward(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{

	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * target_td.dist);
	double v = target_td.v_i;
	if (a == 0.0)
	{
		if (target_v != 0.0) {
			tcp_seed.tt = s / target_v;
		} else {
			tcp_seed.tt = 0.05;
		}
	}
	else if (a > GlobalState::robot_config.maximum_acceleration_forward)
	{
		a = GlobalState::robot_config.maximum_acceleration_forward;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else if (a < -GlobalState::robot_config.maximum_deceleration_forward)
	{
		a = -GlobalState::robot_config.maximum_deceleration_forward;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else {
		tcp_seed.tt = (target_v - target_td.v_i) / a;
	}
	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	if (GlobalState::reverse_planning)
		compute_a_and_t_from_s_reverse(s, target_v, target_td, tcp_seed, params);
	else
		compute_a_and_t_from_s_foward(s, target_v, target_td, tcp_seed, params);
}


void
move_path_to_current_robot_pose(vector<carmen_robot_and_trailer_path_point_t> &path, carmen_robot_and_trailer_pose_t *localizer_pose)
{
	for (std::vector<carmen_robot_and_trailer_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		double x = localizer_pose->x + it->x * cos(localizer_pose->theta) - it->y * sin(localizer_pose->theta);
		double y = localizer_pose->y + it->x * sin(localizer_pose->theta) + it->y * cos(localizer_pose->theta);
		it->x = x;
		it->y = y;
		it->theta = carmen_normalize_theta(it->theta + localizer_pose->theta);
	}
}


carmen_robot_and_trailer_path_point_t
move_to_front_axle(carmen_robot_and_trailer_path_point_t pose)
{
	return (pose);

	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	carmen_robot_and_trailer_path_point_t pose_moved = pose;
	pose_moved.x += L * cos(pose.theta);
	pose_moved.y += L * sin(pose.theta);

	return (pose_moved);
}

//#define NEW_PATH_TO_LANE_DISTANCE

#ifdef NEW_PATH_TO_LANE_DISTANCE

double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;

	carmen_robot_and_trailer_path_point_t nearest_point = {};
	unsigned int i;
	int status;
	for (i = 0; i < (my_params->detailed_lane.size() - 1); i += 1)
	{
		nearest_point = carmen_get_point_nearest_to_path(&status, my_params->detailed_lane[i], my_params->detailed_lane[i + 1], path[0], 0.001);
		if (status == POINT_WITHIN_SEGMENT)
			break;
	}
	if (i >= my_params->detailed_lane.size())
		nearest_point = carmen_get_point_nearest_to_path(&status, my_params->detailed_lane[0], my_params->detailed_lane[1], path[0], 0.001);
	double angle = ANGLE2D(path[0], nearest_point);
	bool first_point_side = (carmen_normalize_theta(path[0].theta - angle) > 0.0)? true: false;

	for (unsigned int i = 0, j = 0; i < path.size(); i += increment)
	{
		for ( ; j < (my_params->detailed_lane.size() - 1); j += 1)
		{
			nearest_point = carmen_get_point_nearest_to_path(&status, my_params->detailed_lane[j], my_params->detailed_lane[j + 1], path[i], 0.001);
			if (status == POINT_WITHIN_SEGMENT)
				break;
		}

		angle = ANGLE2D(path[i], nearest_point);
		bool point_side = (carmen_normalize_theta(path[i].theta - angle) > 0.0)? true: false;
		if (point_side != first_point_side)
			distance = 5.0 * DIST2D(path[i], nearest_point);
		else
			distance = DIST2D(path[i], nearest_point);
		total_distance += distance * distance;
		total_points += 1.0;
	}

	return (total_distance / total_points);
}

#else

double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;
	for (unsigned int i = 0; i < path.size(); i += increment)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			if (GlobalState::reverse_planning)
				distance = DIST2D(path.at(i),
								my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
			else
				distance = DIST2D(move_to_front_axle(path.at(i)),
								my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
			total_points += 1.0;
		}
		else
			distance = 0.0;

		total_distance += distance * distance;
	}

	if (total_points > 0.0)
		return (((total_distance / total_points) > 7.0)? 7.0: total_distance / total_points);
	else
		return (0.0);
}

#endif

vector<carmen_robot_and_trailer_path_point_t>
compute_path_to_lane_distance_evaluation(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;

	vector<carmen_robot_and_trailer_path_point_t> modified_path;
	for (unsigned int i = 0; i < path.size(); i += increment)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			if (GlobalState::reverse_planning)
				modified_path.push_back(path.at(i));
			else
				modified_path.push_back(move_to_front_axle(path.at(i)));
		}
	}

	return (modified_path);
}


void
compute_path_points_nearest_to_lane(ObjectiveFunctionParams *param, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	param->path_point_nearest_to_lane.clear();
	param->path_size = path.size();
	unsigned int index = 0;
	for (unsigned int j = 0; j < path.size(); j++)
	{
		carmen_robot_and_trailer_path_point_t axle;

		if (GlobalState::reverse_planning) //mantem o eixo traseiro
			axle = path.at(j);
		else
			axle = move_to_front_axle(path.at(j));

		double min_dist = DIST2D(axle, param->detailed_lane.at(index));
		for (unsigned int i = index; i < param->detailed_lane.size(); i++)
		{
			double distance = DIST2D(axle, param->detailed_lane.at(i));
			if (distance < min_dist)
			{
				min_dist = distance;
				index = i;
			}
		}
		param->path_point_nearest_to_lane.push_back(index);
	}
}


inline carmen_ackerman_path_point_t
move_path_point_to_map_coordinates(const carmen_ackerman_path_point_t point, double displacement)
{
	carmen_ackerman_path_point_t path_point_in_map_coords;
	double coss, sine;

	sincos(point.theta, &sine, &coss);
	double x_disp = point.x + displacement * coss;
	double y_disp = point.y + displacement * sine;

	sincos(GlobalState::localizer_pose->theta, &sine, &coss);
	path_point_in_map_coords.x = (GlobalState::localizer_pose->x - GlobalState::distance_map->config.x_origin + x_disp * coss - y_disp * sine) / GlobalState::distance_map->config.resolution;
	path_point_in_map_coords.y = (GlobalState::localizer_pose->y - GlobalState::distance_map->config.y_origin + x_disp * sine + y_disp * coss) / GlobalState::distance_map->config.resolution;

	return (path_point_in_map_coords);
}


double
compute_proximity_to_obstacles_using_distance_map(vector<carmen_robot_and_trailer_path_point_t> path)
{
	double proximity_to_obstacles_for_path = 0.0;
	double safety_distance = GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance;

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		carmen_robot_and_trailer_pose_t point_to_check = {path[i].x, path[i].y, path[i].theta, path[i].beta};
		double proximity_point = carmen_obstacle_avoider_proximity_to_obstacles(GlobalState::localizer_pose,
				point_to_check, GlobalState::distance_map, safety_distance);
		proximity_to_obstacles_for_path += proximity_point;
//		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//		getchar();
	}

	return (proximity_to_obstacles_for_path);
}


//double
//get_distance_dependent_activation_factor(double threshold, ObjectiveFunctionParams *my_params)
//{
//	double activation_factor = 1.0;
//
//	if (my_params->target_td->dist < threshold)
//	{
//		if (my_params->target_td->dist > (threshold - 1.0))
//			activation_factor = my_params->target_td->dist - (threshold - 1.0);
//		else
//			activation_factor = 0.0;
//	}
//
//	return (activation_factor);
//}


double
get_beta_activation_factor()
{
	if (GlobalState::semi_trailer_config.type == 0)
		return (0.0);
	else
		return (100.0);
}


double
compute_semi_trailer_to_goal_distance(vector<carmen_robot_and_trailer_path_point_t> path, TrajectoryDimensions *target_td)
{
	if (path.size() == 0)
		return (0.0);

	carmen_robot_and_trailer_path_point_t robot_pose = path[path.size() - 1];
	carmen_robot_and_trailer_pose_t expected_robot_pose = target_td->goal_pose;

	carmen_point_t semi_trailer_pose, expected_semi_trailer_pose;

	semi_trailer_pose.x = robot_pose.x - GlobalState::semi_trailer_config.M * cos(robot_pose.theta) - GlobalState::semi_trailer_config.d * cos(robot_pose.theta - robot_pose.beta);
	semi_trailer_pose.y	= robot_pose.y - GlobalState::semi_trailer_config.M * sin(robot_pose.theta) - GlobalState::semi_trailer_config.d * sin(robot_pose.theta - robot_pose.beta);

	expected_semi_trailer_pose.x = expected_robot_pose.x - GlobalState::semi_trailer_config.M * cos(expected_robot_pose.theta) - GlobalState::semi_trailer_config.d * cos(expected_robot_pose.theta - expected_robot_pose.beta);
	expected_semi_trailer_pose.y = expected_robot_pose.y - GlobalState::semi_trailer_config.M * sin(expected_robot_pose.theta) - GlobalState::semi_trailer_config.d * sin(expected_robot_pose.theta - expected_robot_pose.beta);

	double semi_trailer_to_goal_distance = DIST2D(semi_trailer_pose, expected_semi_trailer_pose);

	return (semi_trailer_to_goal_distance);
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;
	int size = my_params->tcp_seed->k.size();

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(size);
	gsl_vector_memcpy(x_h, v);

	double h = 0.00005;
	double f_x = my_params->my_f(v, params);
	for (int i = 0; i < size; i++)
	{
		gsl_vector_set(x_h, i, gsl_vector_get(v, i) + h);
		double f_x_h = my_params->my_f(x_h, params);
		double d_f_x_h = (f_x_h - f_x) / h;
		gsl_vector_set(df, i, d_f_x_h);
		gsl_vector_set(x_h, i, gsl_vector_get(v, i));
	}

	gsl_vector_free(x_h);
}


void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;
	*f = my_params->my_f(x, params);
	my_df(x, params, df);
}


void
compute_suitable_acceleration_and_tt(ObjectiveFunctionParams &params,
		TrajectoryControlParameters &tcp_seed,
		TrajectoryDimensions target_td, double target_v)
{
	// (i) S = Vo*t + 1/2*a*t^2
	// (ii) dS/dt = Vo + a*t
	// dS/dt = 0 => máximo ou mínimo de S => 0 = Vo + a*t; a*t = -Vo; (iii) a = -Vo/t; (iv) t = -Vo/a
	// Se "a" é negativa, dS/dt = 0 é um máximo de S
	// Logo, como S = target_td.dist, "a" e "t" tem que ser tais em (iii) e (iv) que permitam que
	// target_td.dist seja alcançada.
	//
	// O valor de maxS pode ser computado substituindo (iv) em (i):
	// maxS = Vo*-Vo/a + 1/2*a*(-Vo/a)^2 = -Vo^2/a + 1/2*Vo^2/a = -1/2*Vo^2/a
	//
	// Se estou co velocidade vi e quero chagar a vt, sendo que vt < vi, a eh negativo. O tempo, tt, para
	// ir de vi a vt pode ser derivado de dS/dt = Vo + a*t -> vt = vi + a*tt; a*tt = vt - vi; tt = (vt - vi) / a

	if (!GlobalState::reverse_planning && target_v < 0.0)
		target_v = 0.0;

	tcp_seed.s = target_td.dist; // Pior caso: forcca otimizacao para o primeiro zero da distancia, evitando voltar de reh para atingir a distancia.
	//tcp_seed.valid = true; //@CAIO: adicionei aqui
	compute_a_and_t_from_s(tcp_seed.s, target_v, target_td, tcp_seed, &params);
}


vector<carmen_robot_and_trailer_path_point_t>
move_detailed_lane_to_front_axle(vector<carmen_robot_and_trailer_path_point_t> &detailed_lane)
{
	for (unsigned int i = 0; i < detailed_lane.size(); i++)
		detailed_lane[i] = (move_to_front_axle(detailed_lane[i]));

	return (detailed_lane);
}


double
get_path_to_lane_distance(TrajectoryDimensions td,
		TrajectoryControlParameters tcp, ObjectiveFunctionParams *my_params)
{
	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->beta_i);
	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
	{
		compute_path_points_nearest_to_lane(my_params, path);
		path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
	}
	return (path_to_lane_distance);
}




TrajectoryControlParameters
get_n_knots_tcp_from_detailed_lane(vector<carmen_robot_and_trailer_path_point_t> detailed_lane,
		int n, double v_i, double phi_i, double d_yaw, double a, double s, double tt)
{
	TrajectoryControlParameters tcp = {};
	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;

	tcp.valid = true;
	tcp.a = a;
	tcp.s = s;
	tcp.tt = tt;
	tcp.vf = v_i + tcp.a * tcp.tt;

	unsigned int size = n;

	vector<double> knots_x;
	for (unsigned int i = 0; i < size; i++)
	{
		double x = (tcp.tt / (double) (size - 1)) * (double) i;
		knots_x.push_back(x);
	}
	knots_x[size - 1] = tcp.tt; // Para evitar que erros de arredondamento na conta de x, acima, atrapalhe a leitura do ultimo ponto no spline
	if (size == 4)
	{
		knots_x[1] = tcp.tt / 4.0;
		knots_x[2] = tcp.tt / 2.0;
	}

	tcp.k = {};
	tcp.k.push_back(phi_i);
	if (detailed_lane.size() >= 3)
	{
		unsigned int j = 1;
		double s_consumed = 0.0;
		for (unsigned int i = 1; i < size; i++)
		{
			double t = knots_x[i];
			double s = fabs(v_i * t + 0.5 * tcp.a * t * t);
			unsigned int j_ini = j;
			double s_ini = s_consumed;
			for ( ; j < detailed_lane.size() - 1; j++)
			{
				s_consumed += DIST2D(detailed_lane[j], detailed_lane[j + 1]);
				if (s_consumed > s)
					break;
			}
			double delta_theta = carmen_normalize_theta(detailed_lane[j].theta - detailed_lane[j_ini].theta);
			double phi = atan((L * delta_theta) / (s_consumed - s_ini));

			if (GlobalState::reverse_planning)
				tcp.k.push_back(-carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));
			else
				tcp.k.push_back(carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));
		}

		tcp.sf = s_consumed;
	}
	else
	{
		double s_ini = 0.0;
		double s = 0.0;
		for (unsigned int i = 1; i < size; i++)
		{
			double t = knots_x[i];
			s = fabs(v_i * t + 0.5 * tcp.a * t * t);
			double delta_theta = carmen_normalize_theta((d_yaw / tcp.tt) * t);
			double phi = atan((L * delta_theta) / (s - s_ini));

			if (GlobalState::reverse_planning)
				tcp.k.push_back(-carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));
			else
				tcp.k.push_back(carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));

			s_ini = s;
		}

		tcp.sf = s;
	}

	return (tcp);
}


TrajectoryControlParameters
reduce_tcp_to_3_knots(TrajectoryControlParameters previous_tcp)
{
	TrajectoryControlParameters tcp = previous_tcp;

	if (previous_tcp.k.size() == 4)
	{
		tcp.k[1] = tcp.k[2];
		tcp.k[2] = tcp.k[3];
		tcp.k.pop_back();
	}
	else
	{
		tcp.k = {};
		tcp.k.push_back(previous_tcp.k[0]);
		tcp.k.push_back(previous_tcp.k[tcp.k.size() / 2]);
		tcp.k.push_back(previous_tcp.k[tcp.k.size() - 1]);
	}

	return (tcp);
}


bool
more_knots_required(TrajectoryDimensions target_td)
{
	double threshold_v = GlobalState::param_parking_speed_limit * 1.4;
	if (target_td.v_i > threshold_v)
		return (true);
	else
		return (false);
}


TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryControlParameters previous_tcp,
		TrajectoryDimensions target_td, double target_v,
		vector<carmen_robot_and_trailer_path_point_t> detailed_lane, bool use_lane)
{
	GlobalState::eliminate_path_follower = 0;
	//	virtual_laser_message.num_positions = 0;

	ObjectiveFunctionParams params;
	params.use_lane = use_lane;
//	detailed_lane = {};
	if (detailed_lane.size() > 1)
	{
		if (GlobalState::reverse_planning)
			params.detailed_lane = detailed_lane;
		else
			params.detailed_lane = move_detailed_lane_to_front_axle(detailed_lane);
	}
	else
		params.use_lane = false;

	//int max_iterations;@CAIO: comentei aqui
//	if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK))
	/*if (((GlobalState::semi_trailer_config.type != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN)) ||@CAIO: comentei aqui
		(target_td.dist < GlobalState::distance_between_waypoints / 1.5))
		max_iterations = 150;
	else
		max_iterations = 50;*/

	TrajectoryControlParameters tcp_seed;
	if (!previous_tcp.valid)
	{
		compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);
		//tcp_seed = get_n_knots_tcp_from_detailed_lane(detailed_lane, 3, @CAIO: comentei aqui
		//		target_td.v_i, target_td.phi_i, target_td.d_yaw, tcp_seed.a,  tcp_seed.s, tcp_seed.tt);	// computa tcp com tres nos
	}
	else
	{
		//tcp_seed = reduce_tcp_to_3_knots(previous_tcp); @CAIO: comentei aqui
		compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);
	}

#ifdef PUBLISH_PLAN_TREE
	TrajectoryControlParameters  tcp_copy;
#endif

	// Precisa chamar o otimizador abaixo porque o seguinte (optimized_lane_trajectory_control_parameters()) pode ficar preso em minimo local
	// quando de obstaculos. O otimizador abaixo nao considera obstaculos ou a detailed_lane.
	TrajectoryControlParameters tcp_complete = tcp_seed;//get_optimized_trajectory_control_parameters(tcp_seed, params);//@CAIO: comentei aqui(alterei para tcp_complete = tcp_seed)
#ifdef PUBLISH_PLAN_TREE
	tcp_copy = tcp_complete;
#endif

	//if (!tcp_complete.valid) @CAIO: comentei aqui
	//	return (tcp_complete); @CAIO: comentei aqui

//	if (more_knots_required(target_td))
//	if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK) ||
//		(target_td.dist < GlobalState::distance_between_waypoints / 1.5))
	//if (((GlobalState::semi_trailer_config.type != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN)) ||@CAIO: comentei aqui
	//	(target_td.dist < GlobalState::distance_between_waypoints / 1.5))@CAIO: comentei aqui
	//	get_tcp_with_n_knots(tcp_complete, 3);@CAIO: comentei aqui
	//else@CAIO: comentei aqui
	//	get_tcp_with_n_knots(tcp_complete, 4);@CAIO: comentei aqui

	//tcp_complete = get_optimized_trajectory_control_parameters(tcp_complete, params); @CAIO: comentei aqui

#ifdef PUBLISH_PLAN_TREE
	TrajectoryDimensions td = target_td;
	TrajectoryControlParameters tcp = tcp_complete;
	tcp.valid = true;
	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, td.v_i, td.beta_i);
	print_lane(path, (char *) "caco.txt");
	vector<carmen_robot_and_trailer_path_point_t> path_prev = simulate_car_from_parameters(td, tcp_copy, td.v_i, td.beta_i);

	ObjectiveFunctionParams params_copy = params;
	if (params_copy.detailed_lane.size() > 0)
	{
		Tree tree;
		tree.num_paths = 2;
		tree.num_edges = 0;
		tree.p1 = NULL;
		tree.p2 = NULL;
		tree.paths = (carmen_robot_and_trailer_traj_point_t **) malloc(tree.num_paths * sizeof(carmen_robot_and_trailer_traj_point_t *));
		tree.paths_sizes = (int *) malloc(tree.num_paths * sizeof(int));

		move_path_to_current_robot_pose(path_prev, GlobalState::localizer_pose);
		tree.paths[0] = (carmen_robot_and_trailer_traj_point_t *) malloc(path_prev.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
		copy_path_to_traj(tree.paths[0], path_prev);
		tree.paths_sizes[0] = (path_prev.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: path_prev.size();

//		compute_path_points_nearest_to_lane(&params_copy, path);
//		vector<carmen_robot_and_trailer_path_point_t> modified_path = compute_path_to_lane_distance_evaluation(&params_copy, path);
		vector<carmen_robot_and_trailer_path_point_t> modified_path = path;
		move_path_to_current_robot_pose(modified_path, GlobalState::localizer_pose);
		tree.paths[1] = (carmen_robot_and_trailer_traj_point_t *) malloc(modified_path.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
		copy_path_to_traj(tree.paths[1], modified_path);
		tree.paths_sizes[1] = (modified_path.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: modified_path.size();

//		move_path_to_current_robot_pose(params_copy.detailed_lane, GlobalState::localizer_pose);
//		tree.paths[2] = (carmen_robot_and_trailer_traj_point_t *) malloc(params_copy.detailed_lane.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
//		copy_path_to_traj(tree.paths[2], params_copy.detailed_lane);
//		tree.paths_sizes[2] = (params_copy.detailed_lane.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: params_copy.detailed_lane.size();

		Publisher_Util::publish_plan_tree_message(tree);
	}
#endif
	GlobalState::eliminate_path_follower = 1;

	return (tcp_complete);
}