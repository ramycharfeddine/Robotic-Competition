from protocol import*
from set_up import*


# Define path to follow
def init_path(x_init, y_init):
    # Define list of relative positions: How much the robot has to move from his previous position
    #rel_pos = np.array([[4,0],[-2,2],[2,2],[-4,0],[0,-4]], dtype=float)
    rel_pos = np.array([[0,5.5],[0.75,0],[0,-0.5],[0.75,0],[0,1],[1.5,0],[0,-1],[-1,0],[0,-1],[3,0],
                        [0,-1],[-4,0],[0,-0.75],[5,0],[0,-0.75],[-5,0],[0,-0.75],[5,0],[0,-0.75],[-6,0]],dtype=float)

    # Define initial position of the robot 
    abs_pos = [[x_init,y_init]] # List of lists that will contain the absolute x,y,theta coordinates of the robot

    # Provided the relative positions, compute the absolute positions at each step
    for (dx,dy) in rel_pos[:]:
        (x,y) = abs_pos[-1][0],abs_pos[-1][1]
        d = math.sqrt(dx**2+dy**2)
        new_pose  = [x+dx,y+dy]
        abs_pos.append(new_pose)
    
    abs_pos.pop(0) # remove the first element which corresponds to the initial position
    abs_pos = np.array(abs_pos)
    abs_pos[:,0:1] = abs_pos[:,0:1]*W/8
    abs_pos[:,1:2] = abs_pos[:,1:2]*H/8

    return abs_pos

abs_pos = init_path(1, 1)
plt.figure(figsize=(8,8))
plt.plot(abs_pos[:,0:1], abs_pos[:,1:2], color="r", marker="o")
print(abs_pos)