from controller import Supervisor, Motion
import math, random, os

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
name = robot.getName()
print(f"✅ {name} is online!")

# Walking animation
motions_dir = r'C:\Program Files\Webots\projects\robots\softbank\nao\motions'
forwards = Motion(os.path.join(motions_dir, 'Forwards50.motion'))
forwards.setLoop(True)
forwards.play()

sonar_left  = robot.getDevice('Sonar/Left')
sonar_right = robot.getDevice('Sonar/Right')
sonar_left.enable(timestep)
sonar_right.enable(timestep)

self_node   = robot.getSelf()
trans_field = self_node.getField('translation')
rot_field   = self_node.getField('rotation')

# Get green goal position so they can avoid walking through it 
goal_node        = robot.getFromDef('green_goal')
goal_trans_field = goal_node.getField('translation')

# Avoiding obstacle too 
obstacle_nodes = []
for obs_name in ['obstacle_1', 'obstacle_2', 'obstacle_3']:
    node = robot.getFromDef(obs_name)
    if node is not None:
        obstacle_nodes.append(node)

# Movement settings 
direction   = random.uniform(0, 2 * math.pi)
SPEED       = 0.015
ARENA_LIMIT = 1.7
SONAR_LIMIT = 0.8
GOAL_RADIUS = 0.35    # how close before red robot bounces off green ball
OBS_RADIUS  = 0.25    # how close before red robot bounces off red boxes

print(f"✅ {name} walking!")

# Collision check function 
def would_collide(nx, ny):
    # Check green ball
    gx, gy, gz = goal_trans_field.getSFVec3f()
    if math.sqrt((nx - gx)**2 + (ny - gy)**2) < GOAL_RADIUS:
        return True
    # Check red boxes
    for obs in obstacle_nodes:
        ox, oy, oz = obs.getField('translation').getSFVec3f()
        if math.sqrt((nx - ox)**2 + (ny - oy)**2) < OBS_RADIUS:
            return True
    return False

while robot.step(timestep) != -1:

    x, y, z    = trans_field.getSFVec3f()
    dist_left  = sonar_left.getValue()
    dist_right = sonar_right.getValue()

    near_wall     = abs(x) > ARENA_LIMIT or abs(y) > ARENA_LIMIT
    near_obstacle = dist_left < SONAR_LIMIT or dist_right < SONAR_LIMIT

    if near_wall or near_obstacle:
        direction = (direction + math.pi + random.uniform(-0.6, 0.6)) % (2 * math.pi)

    new_x = x + SPEED * math.cos(direction)
    new_y = y + SPEED * math.sin(direction)
    
    #  bounce off green ball and red boxes 
    if would_collide(new_x, new_y):
        direction = (direction + math.pi + random.uniform(-0.4, 0.4)) % (2 * math.pi)
        new_x = x
        new_y = y

    new_x = max(-ARENA_LIMIT, min(ARENA_LIMIT, new_x))
    new_y = max(-ARENA_LIMIT, min(ARENA_LIMIT, new_y))

    trans_field.setSFVec3f([new_x, new_y, z])
    rot_field.setSFRotation([0, 0, 1, direction])
    self_node.resetPhysics()
