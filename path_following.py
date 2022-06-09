from protocol import*
from set_up import*

 # Define motion
def motion(ard,mode, dist = None, angle = None):
    
    if (mode == "forward"):
        ard.move(Protocol.FORWARD, distance = dist, timeout = 20)
    elif mode == "backward":
        ard.move(Protocol.BACKWARD, distance = dist, timeout = 20)
    elif mode == "turn_right":
        dist = theta_to_dist(angle)
        ard.move(Protocol.RIGHT, distance=dist)
    elif mode == "turn_left":
        dist = theta_to_dist(angle)
        ard.move(Protocol.LEFT, distance=dist)

def theta_to_dist(theta): # theta in rad
    'function to convert the angle of rotation to the distance that has to be travelled in cm'
    dist = theta * (LENGTH_WHEELS*10**(2))
    return dist

# Define path characteristics

def path_parameters(start, goal): # start and goal = 1D array [x,y]
    distance = math.dist(start,goal)
    angle_orientation = math.atan2(goal[1]-start[1],goal[0]-start[0])
    return distance, angle_orientation

def get_angle_to_turn(target_angle, robot_angle): # target_angle, robot_angle in deg
    phi = (robot_angle-target_angle) % 360
    sign = -1
    # used to calculate sign
    if not ((phi >= 0 and phi <= 180) or (
            phi <= -180 and phi >= -360)):
        sign = 1
    if phi > 180:
        result = 360-phi
    else:
        result = phi
    return result*sign

def speed_ms_nominal(speed): # takes a speed in m/s and converts it to nominal scale
    speed_nom = (speed*255)/((7305*np.pi*2*RADIUS)/60)
    # speed_nom = (speed*255)/((2060*np.pi*2*RADIUS)/60)
    return speed_nom

def get_speed_motion(robot_pos, target): # robot_pos from localisation [x,y,theta] // target from path [x,y]
    
    # Tuning parameters: exponentially stable if k_rho>0, k_alpha-k_rho>0
    k_rho = 3
    k_alpha = 5
    
    # Determine rho
    delta_x = target[0] - robot_pos[0]
    delta_y = target[1] - robot_pos[1]
    rho = math.dist(robot_pos[0:2],target)
    
    # Determine alpha
    alpha = math.radians(abs(get_angle_to_turn(math.atan2(delta_y, delta_x), robot_pos[2])))
    
    # Deduce needed speed for rotation and straight motion
    v = k_rho*rho     # [m/s]
    w = k_alpha*alpha # [rad/s]

    # convert speed from m/s to nominal
    v_nom = speed_ms_nominal(v)
    w_nom = speed_ms_nominal(RADIUS*w)

    return v_nom, w_nom

def follow_path(ard,robot_pos, target): # robot_pos from localisation [x,y,theta] // target from path [x,y] both global variables
    dist_target, angle_target = path_parameters(robot_pos[0:2], target)

    angle_target_deg = math.degrees(angle_target)
    angle_robot_deg  = math.degrees(robot_pos[2])
    
    angle_rotation = math.radians(get_angle_to_turn(angle_target_deg, angle_robot_deg)) # angle the robot has to turn with to reach the goal
    
    #v, w = get_speed_motion(robot_pos, target)

    if abs(angle_rotation) > EPS_THETA:
        print("aas")
        if angle_rotation > 0:
            motion(ard,mode = "turn_left", angle = abs(angle_rotation))
        elif angle_rotation < 0:
            motion(ard,mode = "turn_right", angle = abs(angle_rotation))
            
    elif dist_target > EPS_DIST:
        motion(ard,mode = "forward", dist = dist_target)
        
def goal_reached(robot_pos, target): # here robot_pos [x,y,theta] and target [x,y] 
    if math.dist(robot_pos[0:2], target) <= EPS_DIST:
        return True
    else:
        return False