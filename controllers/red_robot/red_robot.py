from controller import Supervisor, Motion
import math, random, os

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
name = robot.getName()
print(f"✅ {name} is online!")

#  Walking animation (
motions_dir = r'C:\Program Files\Webots\projects\robots\softbank\nao\motions'
forwards = Motion(os.path.join(motions_dir, 'Forwards50.motion'))
forwards.setLoop(True)
forwards.play()

#  Sonar sensors 
sonar_left  = robot.getDevice('Sonar/Left')
sonar_right = robot.getDevice('Sonar/Right')
sonar_left.enable(timestep)
sonar_right.enable(timestep)

#  Get red robot's own position in the world 
self_node   = robot.getSelf()
trans_field = self_node.getField('translation')
rot_field   = self_node.getField('rotation')

#  Movement settings 
direction   = random.uniform(0, 2 * math.pi)  # random starting direction
SPEED       = 0.015    # metres moved per timestep
ARENA_LIMIT = 1.7      # half the arena size (arena is 4x4, so limit is 1.8)
SONAR_LIMIT = 0.8      # turn if sonar sees something within 0.8m

print(f"✅ {name} walking!")

while robot.step(timestep) != -1:

    # Read current position
    x, y, z = trans_field.getSFVec3f()

    # Read sonar
    dist_left  = sonar_left.getValue()
    dist_right = sonar_right.getValue()

    # Check if near a wall or obstacle
    near_wall     = abs(x) > ARENA_LIMIT or abs(y) > ARENA_LIMIT
    near_obstacle = dist_left < SONAR_LIMIT or dist_right < SONAR_LIMIT

    if near_wall or near_obstacle:
        # Flip direction amd add some randomness so it doesn't get stuck
        direction = (direction + math.pi + random.uniform(-0.6, 0.6)) % (2 * math.pi)

    # Calculate new position
    new_x = x + SPEED * math.cos(direction)
    new_y = y + SPEED * math.sin(direction)

    # Hard clamp, the robot can never escape the arena
    new_x = max(-ARENA_LIMIT, min(ARENA_LIMIT, new_x))
    new_y = max(-ARENA_LIMIT, min(ARENA_LIMIT, new_y))

    # Apply position and rotation directly making sure they don't fight physics
    trans_field.setSFVec3f([new_x, new_y, z])
    rot_field.setSFRotation([0, 0, 1, direction])
    self_node.resetPhysics()