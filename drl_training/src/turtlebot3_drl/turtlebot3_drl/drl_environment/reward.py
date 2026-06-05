from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM, \
    BETA_ANGULAR, OBSTACLE_PROXIMITY_DIST, SPEED_LINEAR_MAX

goal_dist_initial = 0

reward_function_internal = None

def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance):
    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)

def get_reward_A(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist):
        # Heading alignment toward goal: [-0.314, 0]
        r_yaw = -0.1 * abs(goal_angle)

        # Angular-velocity penalty for motion stabilization: -(beta * w^2)
        r_vangular = -1 * BETA_ANGULAR * (action_angular ** 2)

        # Distance progress toward goal: [-1, 1]
        r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1

        # Soft obstacle-proximity penalty
        if min_obstacle_dist < OBSTACLE_PROXIMITY_DIST:
            r_obstacle = -2.0
        else:
            r_obstacle = 0.0

        # Encourage forward motion up to the max linear speed (small magnitude)
        r_vlinear = -0.5 * ((SPEED_LINEAR_MAX - action_linear) ** 2)

        reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular

        if succeed == SUCCESS:
            reward += 10.0
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
            reward -= 10.0
        return float(reward)

# Define your own reward function by defining a new function: 'get_reward_X'
# Replace X with your reward function name and configure it in settings.py

def reward_initalize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")
