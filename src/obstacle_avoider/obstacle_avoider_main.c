#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/task_manager_interface.h>
#include "obstacle_avoider_messages.h"
#include "obstacle_avoider_interface.h"
#include "obstacle_avoider.h"

//#define USE_MAIN_LOOP_TO_PUBLISH

static int necessary_maps_available = 0;

static int current_motion_command_vetor = 0;
static carmen_robot_and_trailer_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS][NUM_MOTION_COMMANDS_PER_VECTOR];
static int num_motion_commands_in_vector[NUM_MOTION_COMMANDS_VECTORS];
static double timestamp_of_motion_commands_vector[NUM_MOTION_COMMANDS_VECTORS];

static carmen_robot_ackerman_config_t carmen_robot_ackerman_config;
static carmen_semi_trailer_config_t carmen_semi_trailer_config;
static double carmen_robot_ackerman_collision_avoidance_frequency;
static double carmen_robot_ackerman_sensor_time_of_last_update = -1.0;
static double carmen_robot_ackerman_motion_command_time_of_last_update = -1.0;
static double last_motion_command_total_time = 0.0;

static char *robot_ackerman_host;

static int ackerman_collision_avoidance;
static double robot_sensor_timeout;
static double command_timeout;

static carmen_behavior_selector_algorithm_t current_algorithm = CARMEN_BEHAVIOR_SELECTOR_GRADIENT;
static carmen_behavior_selector_task_t current_task = BEHAVIOR_SELECTOR_PARK;

static int use_truepos = 0;
static int log_mode = 0;

static double last_behaviour_selector_compact_lane_contents_message_timestamp = 0.0;

static int argc_global;
static char **argv_global;

//extern carmen_mapper_virtual_laser_message virtual_laser_message;
//#define MAX_VIRTUAL_LASER_SAMPLES 10000

int eliminate_path_follower = 1;
double eliminate_path_follower_transition_v = 4.16666;
double robot_velocity_delay = 0.46;
double robot_min_v_distance_ahead = 0.0;
double robot_steering_delay = 0.26;
double robot_min_s_distance_ahead = 0.1;


static void
consume_motion_command_time(int motion_command_vetor)
{
	if (log_mode)
		return;

	int i, j;
	double current_time, time_consumed;

	current_time = carmen_get_time();
	time_consumed = carmen_robot_ackerman_motion_command_time_of_last_update;
	for (i = 0; i < num_motion_commands_in_vector[motion_command_vetor]; i++)
	{
		time_consumed += motion_commands_vector[motion_command_vetor][i].time;

		if (time_consumed > current_time)
			break;
	}

	if (i < num_motion_commands_in_vector[motion_command_vetor])
	{
		motion_commands_vector[motion_command_vetor][0] = motion_commands_vector[motion_command_vetor][i];
		motion_commands_vector[motion_command_vetor][0].time = time_consumed - current_time;
		for (j = 1; (j + i) < num_motion_commands_in_vector[motion_command_vetor]; j++)
			motion_commands_vector[motion_command_vetor][j] = motion_commands_vector[motion_command_vetor][j + i];

		num_motion_commands_in_vector[motion_command_vetor] = j;
		carmen_robot_ackerman_motion_command_time_of_last_update = current_time;
	}
	else
	{
		num_motion_commands_in_vector[motion_command_vetor] = 1;
		motion_commands_vector[motion_command_vetor][0].v = 0.0;
		motion_commands_vector[motion_command_vetor][0].time = 1.0;
	}
}


//static void
//print_path(carmen_robot_and_trailer_motion_command_t *path, int size)
//{
//	FILE *caco = fopen("caco2.txt", "a");
//
//	printf("\n*** size %d, s %.3lf, timestamp %lf\n", size, DIST2D(path[0], path[size - 1]), carmen_get_time());
//	fprintf(caco, "\n*** size %d, s %.3lf, timestamp %lf\n", size, DIST2D(path[0], path[size - 1]), carmen_get_time());
//	for (int i = 0; i < size; i++)
//	{
//		if (i < ((size > 10)? 10: size))
//			printf(" i %2d, x %.2f, y %.2f, phi %.2f, theta %.2f, v[i] %lf, dist_2D [i]->[i+1] %.2f, dt %.3lf\n", i,
//				path[i].x, path[i].y, path[i].phi, //carmen_radians_to_degrees(path[i].phi),
//				carmen_radians_to_degrees(path[i].theta),
//				path[i].v,
//				(i < size - 1)? DIST2D(path[i], path[i + 1]): 0.0, path[i].time); // O displacement eh o deslocamento para chegar ao proximo ponto.
//
//		fprintf(caco, " i %2d, x %.2f, y %.2f, phi %.2f, theta %.2f, v[i] %lf, dist_2D [i]->[i+1] %.2f, dt %.3lf\n", i,
//				path[i].x, path[i].y, path[i].phi, //carmen_radians_to_degrees(path[i].phi),
//				carmen_radians_to_degrees(path[i].theta),
//				path[i].v,
//				(i < size - 1)? DIST2D(path[i], path[i + 1]): 0.0, path[i].time);
//	}
//	fclose(caco);
//}


int
apply_robot_delays(carmen_robot_and_trailer_motion_command_t *original_path, int original_size)
{
	carmen_robot_and_trailer_motion_command_t *path = original_path;
	int size = original_size;

	// Velocity delay
	double time_delay = 0.0;
	double distance_travelled = 0.0;
	int i = 0;
	while ((time_delay < robot_velocity_delay) && (size > 1))
	{
		time_delay += path[0].time;
		distance_travelled += DIST2D(path[0], path[1]);
		path = &(path[1]);
		i++;
		size--;
	}

	while ((distance_travelled < robot_min_v_distance_ahead) && (size > 1))
	{
		distance_travelled += DIST2D(path[0], path[1]);
		path = &(path[1]);
		i++;
		size--;
	}

	for (int j = 0; j < size; j++)
		original_path[j].v = path[j].v;
	for (int j = size - 1; j < original_size; j++)
		original_path[j].v = path[size - 1].v;

	int size_decrease_due_to_velocity_delay = i;

	// Steering delay
	path = original_path;
	size = original_size;
	time_delay = 0.0;
	distance_travelled = 0.0;
	i = 0;
	while ((time_delay < robot_steering_delay) && (size > 1))
	{
		time_delay += path[0].time;
		distance_travelled += DIST2D(path[0], path[1]);
		path = &(path[1]);
		i++;
		size--;
	}

	while ((distance_travelled < robot_min_s_distance_ahead) && (size > 1))
	{
		distance_travelled += DIST2D(path[0], path[1]);
		path = &(path[1]);
		i++;
		size--;
	}

	for (int j = 0; j < size; j++)
		original_path[j].phi = path[j].phi;
	for (int j = size - 1; j < original_size; j++)
		original_path[j].phi = path[size - 1].phi;

	int size_decrease_due_to_steering_delay = i;

	int size_decrease = (size_decrease_due_to_velocity_delay > size_decrease_due_to_steering_delay) ?
							size_decrease_due_to_velocity_delay : size_decrease_due_to_steering_delay;

	return (original_size - size_decrease);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
obstacle_avoider_publish_base_ackerman_motion_command(carmen_robot_and_trailer_motion_command_t *motion_commands,
		int num_motion_commands, double timestamp)
{
	if (eliminate_path_follower)
	{
		carmen_robot_and_trailer_motion_command_t *motion_commands_copy = (carmen_robot_and_trailer_motion_command_t *) malloc(num_motion_commands * sizeof(carmen_robot_and_trailer_motion_command_t));
		memcpy(motion_commands_copy, motion_commands, num_motion_commands * sizeof(carmen_robot_and_trailer_motion_command_t));

		apply_robot_delays(motion_commands_copy, num_motion_commands);
		carmen_obstacle_avoider_publish_base_ackerman_motion_command(motion_commands_copy, num_motion_commands, timestamp);

		free(motion_commands_copy);
	}
	else
		carmen_obstacle_avoider_publish_base_ackerman_motion_command(motion_commands, num_motion_commands, timestamp);
}


static void
publish_navigator_ackerman_plan_message_with_obstacle_avoider_path(carmen_robot_and_trailer_motion_command_t *motion_commands_vector,
		int num_motion_commands, double timestamp)
{
	return;
	carmen_navigator_ackerman_plan_message msg;

	if (num_motion_commands > 0)
	{
		msg = build_navigator_ackerman_plan_message(motion_commands_vector, &num_motion_commands, &carmen_robot_ackerman_config, &carmen_semi_trailer_config, timestamp);
		carmen_obstacle_avoider_publish_path(msg);

		free(msg.path);
	}
}


void
publish_navigator_ackerman_plan_message_with_motion_planner_path(carmen_robot_and_trailer_motion_command_t *motion_commands_vector,
		int num_motion_commands, double timestamp)
{
	return;
	carmen_navigator_ackerman_plan_message msg;

	if (num_motion_commands > 0)
	{
		msg = build_navigator_ackerman_plan_message(motion_commands_vector, &num_motion_commands, &carmen_robot_ackerman_config, &carmen_semi_trailer_config, timestamp);
		carmen_obstacle_avoider_publish_motion_planner_path(msg);

		free(msg.path);
	}
}


void
publish_base_ackerman_motion_command_message_to_stop_robot()
{
	int i;
	int j;

	for (i = 0; i < NUM_MOTION_COMMANDS_VECTORS; i++)
	{
		for (j = 0; j < 2; j++)
		{
			motion_commands_vector[i][j].x = 0.0;
			motion_commands_vector[i][j].y = 0.0;
			motion_commands_vector[i][j].v = 0.0;
			motion_commands_vector[i][j].time = 1.0;
			motion_commands_vector[i][j].phi = get_current_pose().phi;
		}
		num_motion_commands_in_vector[i] = 2;
		timestamp_of_motion_commands_vector[i] = carmen_get_time();
	}

	current_motion_command_vetor = 0;
	carmen_obstacle_avoider_publish_base_ackerman_motion_command(motion_commands_vector[current_motion_command_vetor],
			num_motion_commands_in_vector[current_motion_command_vetor], timestamp_of_motion_commands_vector[current_motion_command_vetor]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


// Esta funcao consome as trajetorias (vetor de v, phi, t) recebidas por robot_ackerman_motion_command_message_handler()
// (na verdade, a última recebida)
void
obstacle_avoider_timer_handler()
{
	// Este eh o handler principal: ele realiza a funcionalidade do modulo
	static double time_of_last_call = 0.0;
	static double time_of_last_heartbeat = 0.0;
	static int robot_hit_obstacle = 0;

	if (!necessary_maps_available)
		return;

	int motion_command_vetor = current_motion_command_vetor;
	consume_motion_command_time(motion_command_vetor);

	if (ackerman_collision_avoidance)
		robot_hit_obstacle |= obstacle_avoider(motion_commands_vector[motion_command_vetor], &(num_motion_commands_in_vector[motion_command_vetor]), &carmen_robot_ackerman_config, &carmen_semi_trailer_config);

	if (num_motion_commands_in_vector[motion_command_vetor] > 0)
	{
		obstacle_avoider_publish_base_ackerman_motion_command(motion_commands_vector[motion_command_vetor],
				num_motion_commands_in_vector[motion_command_vetor], timestamp_of_motion_commands_vector[motion_command_vetor]);

		//  Para informar ao pipeline acima sobre a deteccao de obstaculos (ou nao)
		carmen_obstacle_avoider_publish_robot_hit_obstacle_message(robot_hit_obstacle);

		// Apenas para visualizacao (path vermelho)
		if (ackerman_collision_avoidance && ((carmen_get_time() - time_of_last_call) > 0.2))
		{
			robot_hit_obstacle = 0;
			publish_navigator_ackerman_plan_message_with_obstacle_avoider_path(motion_commands_vector[motion_command_vetor],
					num_motion_commands_in_vector[motion_command_vetor], timestamp_of_motion_commands_vector[motion_command_vetor]);
			time_of_last_call = carmen_get_time();
		}

		if (ackerman_collision_avoidance && ((carmen_get_time() - time_of_last_heartbeat) > 0.1))
		{
			carmen_publish_heartbeat("obstacle_avoider");
			time_of_last_heartbeat = carmen_get_time();
		}
//		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//		virtual_laser_message.num_positions = 0;
	}
}


void
check_message_absence_timeout_timer_handler(void)
{
	if (log_mode)
		return;

	if ((carmen_robot_ackerman_sensor_time_of_last_update >= 0.0) &&
	    (carmen_get_time() - carmen_robot_ackerman_sensor_time_of_last_update) > robot_sensor_timeout)
		publish_base_ackerman_motion_command_message_to_stop_robot();

	if ((carmen_robot_ackerman_motion_command_time_of_last_update >= 0.0) &&
	    ((carmen_get_time() - carmen_robot_ackerman_motion_command_time_of_last_update) - last_motion_command_total_time) > command_timeout)
		publish_base_ackerman_motion_command_message_to_stop_robot();
}


// Esta funcao recebe trajetorias (vetor de v, phi, t) do pipeline acima
static void
robot_ackerman_motion_command_message_handler(carmen_robot_ackerman_motion_command_message *motion_command_message)
{
//	print_path(motion_command_message->motion_command, motion_command_message->num_motion_commands);

	static double time_of_last_call = 0.0;
	carmen_robot_and_trailer_motion_command_t *next_motion_command_vector;
	int i, num_motion_commands;

	if (motion_command_message->num_motion_commands < 1)
		return;

	if (!necessary_maps_available)
		return;

	next_motion_command_vector = motion_commands_vector[(current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS];

	if (motion_command_message->num_motion_commands < NUM_MOTION_COMMANDS_PER_VECTOR)
		num_motion_commands = motion_command_message->num_motion_commands;
	else
		num_motion_commands = NUM_MOTION_COMMANDS_PER_VECTOR;

	for (i = 0; i < num_motion_commands; i++)
		next_motion_command_vector[i] = motion_command_message->motion_command[i];

	num_motion_commands_in_vector[(current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS] = num_motion_commands;
	timestamp_of_motion_commands_vector[(current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS] = motion_command_message->timestamp;
	current_motion_command_vetor = (current_motion_command_vetor + 1) % NUM_MOTION_COMMANDS_VECTORS;

	// Apenas para a visualizacao da mensagem recebida (path verde)
	if (ackerman_collision_avoidance && ((carmen_get_time() - time_of_last_call) > 0.2))
	{
		publish_navigator_ackerman_plan_message_with_motion_planner_path(next_motion_command_vector,
				num_motion_commands, motion_command_message->timestamp);
		time_of_last_call = carmen_get_time();
	}

	carmen_robot_ackerman_motion_command_time_of_last_update = motion_command_message->timestamp;

	last_motion_command_total_time = get_last_motion_command_total_time(next_motion_command_vector, num_motion_commands);

#ifndef USE_MAIN_LOOP_TO_PUBLISH
	obstacle_avoider_timer_handler();
#endif
}


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_robot_and_trailer_traj_point_t pose;

	if (!necessary_maps_available)
		return;

	pose.x = msg->globalpos.x;
	pose.y = msg->globalpos.y;
	pose.theta = msg->globalpos.theta;
	pose.beta = msg->beta;
	pose.v = msg->v;
	pose.phi = msg->phi;

	add_pose_to_pose_vector(pose);

	carmen_robot_ackerman_sensor_time_of_last_update = msg->timestamp;

	if (msg->semi_trailer_type != carmen_semi_trailer_config.type)
	{
		carmen_task_manager_read_semi_trailer_parameters(&carmen_semi_trailer_config, argc_global, argv_global, msg->semi_trailer_type);
		carmen_collision_detection_set_semi_trailer_type(carmen_semi_trailer_config.type);
	}
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	carmen_robot_and_trailer_traj_point_t pose;

	if (!necessary_maps_available)
		return;

	pose.x = msg->truepose.x;
	pose.y = msg->truepose.y;
	pose.theta = msg->truepose.theta;
	pose.beta = msg->beta;
	pose.v = msg->v;
	pose.phi = msg->phi;

	add_pose_to_pose_vector(pose);

	carmen_robot_ackerman_sensor_time_of_last_update = msg->timestamp;
}


//static void
//map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
//{
//	static carmen_compact_map_t *compact_cost_map = NULL;
//	static carmen_map_t cost_map;
//
//	if (compact_cost_map == NULL)
//	{
//		carmen_grid_mapping_create_new_map(&cost_map, message->config.x_size, message->config.y_size, message->config.resolution);
//		memset(cost_map.complete_map, 0, cost_map.config.x_size * cost_map.config.y_size * sizeof(double));
//
//		compact_cost_map = (carmen_compact_map_t*) (calloc(1, sizeof(carmen_compact_map_t)));
//		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
//		carmen_prob_models_uncompress_compact_map(&cost_map, compact_cost_map);
//	}
//	else
//	{
//		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, compact_cost_map, 0.0);
//		carmen_prob_models_free_compact_map(compact_cost_map);
//		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
//		carmen_prob_models_uncompress_compact_map(&cost_map, compact_cost_map);
//	}
//
//	cost_map.config = message->config;
//
//	add_cost_map_to_map_vector(&cost_map);
//
//	necessary_maps_available = 1;
//}


//static void
//carmen_obstacle_distance_mapper_message_handler(carmen_obstacle_distance_mapper_map_message *message)
//{
//	obstacle_avoider_update_map(message);
//
//	necessary_maps_available = 1;
//}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
	static carmen_obstacle_distance_mapper_map_message distance_map;

	if (compact_distance_map == NULL)
	{
		carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(&distance_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}

	obstacle_avoider_update_map(&distance_map);

	necessary_maps_available = 1;
}


static void
carmen_obstacle_distance_mapper_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	carmen_obstacle_distance_mapper_map_message *distance_map = get_current_map();
	if ((distance_map != NULL) && ((message->timestamp - last_behaviour_selector_compact_lane_contents_message_timestamp) > 0.5))
		carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(distance_map, message);
}


static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	carmen_obstacle_distance_mapper_map_message *distance_map = get_current_map();
	if (distance_map != NULL)
		carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(distance_map, message);

	last_behaviour_selector_compact_lane_contents_message_timestamp = message->timestamp;
}


//static void
//grid_mapping_message_handler(carmen_grid_mapping_message *message)
//{
//	add_map_to_map_vector(message);
//
//	necessary_maps_available = 1;
//}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	current_algorithm = msg->algorithm;
	current_task = msg->task;

	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY)
		carmen_collision_detection_set_robot_collision_config(ENGAGE_GEOMETRY);
	else
		carmen_collision_detection_set_robot_collision_config(DEFAULT_GEOMETRY);
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	if (fabs(msg->v) < eliminate_path_follower_transition_v)
		eliminate_path_follower = 1;
	else
		eliminate_path_follower = 0;
}


void
shutdown_obstacle_avoider(int signo __attribute__ ((unused)))
{
	if (signo == SIGINT)
	{
		publish_base_ackerman_motion_command_message_to_stop_robot();
		carmen_ipc_disconnect();
		carmen_warn("\nDisconnected.\n");

		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static int
initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);

	carmen_robot_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) robot_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_grid_mapping_subscribe_message(NULL, (carmen_handler_t) grid_mapping_message_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_map_server_subscribe_compact_cost_map(NULL, (carmen_handler_t) map_server_compact_cost_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_obstacle_distance_mapper_subscribe_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_obstacle_distance_mapper_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

	return (err);
}

static int
read_parameters(int argc, char **argv)
{
	int num_items;
	carmen_param_t param_list[] =
	{
		{"obstacle_avoider", "obstacles_safe_distance", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.obstacle_avoider_obstacles_safe_distance, 1, NULL},
		{"robot", "max_velocity", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.max_v, 1, NULL},
		{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.max_phi, 1, NULL},
		{"robot", "min_approach_dist", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.approach_dist, 1, NULL},
		{"robot", "min_side_dist", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.side_dist, 1, NULL},
		{"robot", "length", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.length, 0, NULL},
		{"robot", "width", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.width, 0, NULL},
		{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.maximum_acceleration_forward, 1, NULL},
		{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.maximum_deceleration_forward, 1, NULL},
		{"robot", "reaction_time", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.reaction_time, 0, NULL},
		{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_rear_wheels, 1,NULL},
		{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_front_and_rear_axles, 1, NULL},
		{"robot", "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_front_car_and_front_wheels, 1, NULL},
		{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_rear_car_and_rear_wheels, 1, NULL},

		{(char *) "robot", (char *) "desired_decelaration_forward",	 CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.desired_decelaration_forward,					1, NULL},
		{(char *) "robot", (char *) "desired_decelaration_reverse",	 CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.desired_decelaration_reverse,					1, NULL},
		{(char *) "robot", (char *) "desired_acceleration",			 CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.desired_acceleration,							1, NULL},
		{(char *) "robot", (char *) "desired_steering_command_rate", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.desired_steering_command_rate,					1, NULL},
		{(char *) "robot", (char *) "understeer_coeficient",		 CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.understeer_coeficient,							1, NULL},
		{(char *) "robot", (char *) "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.maximum_steering_command_rate, 					1, NULL},

		{"robot", "allow_rear_motion", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.allow_rear_motion, 1, NULL},
		{"robot", "sensor_timeout", CARMEN_PARAM_DOUBLE, &robot_sensor_timeout, 1, NULL},
		{"robot", "command_timeout", CARMEN_PARAM_DOUBLE, &command_timeout, 1, NULL},
		{"robot", "collision_avoidance", CARMEN_PARAM_ONOFF, &ackerman_collision_avoidance, 1, NULL},
		{"robot", "collision_avoidance_frequency", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_collision_avoidance_frequency, 1, NULL},
		{"robot", "interpolate_odometry", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.interpolate_odometry, 1, NULL},
		{"semi_trailer", "initial_type", CARMEN_PARAM_INT, &carmen_semi_trailer_config.type, 0, NULL},
		{"behavior_selector", "use_truepos", CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL},
		{"rrt", "log_mode", CARMEN_PARAM_ONOFF,	&log_mode, 1, NULL},

		{(char *) "model",	 (char *) "predictive_planner_eliminate_path_follower",					CARMEN_PARAM_ONOFF,	 &eliminate_path_follower,									 1, NULL},
		{(char *) "model",	 (char *) "predictive_planner_eliminate_path_follower_transition_v",    CARMEN_PARAM_DOUBLE, &eliminate_path_follower_transition_v,					 1, NULL},
		{(char *) "model",	 (char *) "predictive_planner_robot_velocity_delay",                    CARMEN_PARAM_DOUBLE, &robot_velocity_delay,									 1, NULL},
		{(char *) "model",	 (char *) "predictive_planner_robot_min_v_distance_ahead",              CARMEN_PARAM_DOUBLE, &robot_min_v_distance_ahead,								 1, NULL},
		{(char *) "model",	 (char *) "predictive_planner_robot_steering_delay",                    CARMEN_PARAM_DOUBLE, &robot_steering_delay,									 1, NULL},
		{(char *) "model",	 (char *) "predictive_planner_robot_min_s_distance_ahead",              CARMEN_PARAM_DOUBLE, &robot_min_s_distance_ahead,								 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);


	if (carmen_semi_trailer_config.type > 0)
		carmen_task_manager_read_semi_trailer_parameters(&carmen_semi_trailer_config, argc, argv, carmen_semi_trailer_config.type);

	return (0);
}


double
carmen_obstacle_avoider_initialize(int argc, char **argv)
{
	robot_ackerman_host = carmen_get_host();

	initialize_map_vector(NUM_MAPS);

	memset(num_motion_commands_in_vector, 0, NUM_MOTION_COMMANDS_VECTORS * sizeof(int));
	memset(timestamp_of_motion_commands_vector, 0, NUM_MOTION_COMMANDS_VECTORS * sizeof(double));

	if (read_parameters(argc, argv) < 0)
		carmen_die("Could not read the parameters properly in carmen_obstacle_avoider_initialize()");

	if (initialize_ipc() != IPC_OK)
		carmen_die("Error: could not connect to IPC Server in carmen_obstacle_avoider_initialize()\n");

	return (carmen_robot_ackerman_collision_avoidance_frequency);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int 
main(int argc, char **argv)
{
	argc_global = argc;
	argv_global = argv;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_obstacle_avoider);

#ifdef USE_MAIN_LOOP_TO_PUBLISH
	double carmen_obstacle_avoider_collision_avoidance_frequency;
	carmen_obstacle_avoider_collision_avoidance_frequency = carmen_obstacle_avoider_initialize(argc, argv);

	while (1)
	{
		obstacle_avoider_timer_handler();
		check_message_absence_timeout_timer_handler();
		carmen_ipc_sleep(1.0 / carmen_obstacle_avoider_collision_avoidance_frequency);
	}

#else
	carmen_obstacle_avoider_initialize(argc, argv);

//	double carmen_obstacle_avoider_collision_avoidance_frequency;
//	carmen_obstacle_avoider_collision_avoidance_frequency = carmen_obstacle_avoider_initialize(argc, argv);

//	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
//	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
//	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
//	virtual_laser_message.host = carmen_get_host();

//	carmen_ipc_addPeriodicTimer(1.0 / carmen_obstacle_avoider_collision_avoidance_frequency, (TIMER_HANDLER_TYPE) obstacle_avoider_timer_handler, NULL);
//	carmen_ipc_addPeriodicTimer(1.0 / carmen_obstacle_avoider_collision_avoidance_frequency, (TIMER_HANDLER_TYPE) check_message_absence_timeout_timer_handler, NULL);

	carmen_ipc_dispatch();
#endif

	return 0;
}
