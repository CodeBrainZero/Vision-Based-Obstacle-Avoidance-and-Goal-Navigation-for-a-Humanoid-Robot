from controller import Supervisor, Motion, Camera
import math, os, random

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
print("✅ Main robot is online — final version!")

#  Motion files 
motions_dir = r'C:\Program Files\Webots\projects\robots\softbank\nao\motions'
forwards = Motion(os.path.join(motions_dir, 'Forwards50.motion'))
forwards.setLoop(True)
forwards.play()

#  Sensors 
sonar_left  = robot.getDevice('Sonar/Left')
sonar_right = robot.getDevice('Sonar/Right')
sonar_left.enable(timestep)
sonar_right.enable(timestep)

#  Camera 
camera = robot.getDevice('CameraTop')
camera.enable(timestep)
cam_width  = camera.getWidth()
cam_height = camera.getHeight()
print(f"📷 Camera ready: {cam_width}x{cam_height}")

#  Supervisor own the position + goal 
self_node        = robot.getSelf()
trans_field      = self_node.getField('translation')
rot_field        = self_node.getField('rotation')
goal_node        = robot.getFromDef('green_goal')
goal_trans_field = goal_node.getField('translation')

#  obstacle nodes 
obstacle_nodes = []
for name in ['obstacle_1', 'obstacle_2', 'obstacle_3']:
    node = robot.getFromDef(name)
    if node is not None:
        obstacle_nodes.append(node)
        print(f"✅ Found obstacle: {name}")
    else:
        print(f"⚠️  Could not find: {name} — check DEF name in .wbt file")

#  Movement settings 
direction     = 0.0
SPEED         = 0.015
ARENA_LIMIT   = 1.7
SONAR_LIMIT   = 0.8
stuck_counter = 0
STUCK_LIMIT   = 40

#  Colour settings 
RED_THRESHOLD   = 8    
GREEN_THRESHOLD = 5
CHANNEL_HIGH    = 140
CHANNEL_LOW     = 80

#  Goal tracking 
GOAL_REACH_DIST     = 0.4
OBSTACLE_RADIUS     = 0.25
goals_scored        = 0
celebrating         = False
celebrate_timer     = 0
CELEBRATE_STEPS     = 120
goal_cooldown       = 0
GOAL_COOLDOWN_STEPS = 150   # had to add this because it was catching green bally too fast 

#  Debug 
debug_counter = 0
DEBUG_EVERY   = 50

print("✅ Main robot walking! Hunting for the green goal...")

#  Camera scanner 
def scan_camera(image, width, height):
    red_left = red_right = green_left = green_right = 0
    mid = width // 2
    for x in range(0, width, 4):
        for y in range(0, height, 4):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            if r > CHANNEL_HIGH and g < CHANNEL_LOW and b < CHANNEL_LOW:
                if x < mid: red_left += 1
                else:        red_right += 1
            elif g > CHANNEL_HIGH and r < CHANNEL_LOW and b < CHANNEL_LOW:
                if x < mid: green_left += 1
                else:        green_right += 1
    return red_left, red_right, green_left, green_right

#  Obstacle collision check 
def collides_with_obstacle(nx, ny):
    for obs in obstacle_nodes:
        ox, oy, oz = obs.getField('translation').getSFVec3f()
        if math.sqrt((nx - ox)**2 + (ny - oy)**2) < OBSTACLE_RADIUS:
            return True
    return False

#  Respawn goal at a safe random position 
def respawn_goal():
    rx, ry, rz = trans_field.getSFVec3f()
    for _ in range(100):
        new_x = random.uniform(-1.3, 1.3)
        new_y = random.uniform(-1.3, 1.3)
        # it should not be inside obstacle AND must be far enough from robot
        dist_from_robot = math.sqrt((new_x - rx)**2 + (new_y - ry)**2)
        if not collides_with_obstacle(new_x, new_y) and dist_from_robot > 0.8:
            goal_trans_field.setSFVec3f([new_x, new_y, 0.3])
            print(f"🌀 Goal respawned at X:{new_x:.2f} Y:{new_y:.2f} — hunt again!")
            return
    goal_trans_field.setSFVec3f([0.0, 0.0, 0.3])
    print("🌀 Goal respawned at centre!")

#  adding distance towards the goal 
def distance_to_goal():
    rx, ry, rz = trans_field.getSFVec3f()
    gx, gy, gz = goal_trans_field.getSFVec3f()
    return math.sqrt((rx - gx)**2 + (ry - gy)**2)

#  Main loop 
while robot.step(timestep) != -1:

    x, y, z    = trans_field.getSFVec3f()
    dist_left  = sonar_left.getValue()
    dist_right = sonar_right.getValue()

    image = camera.getImage()
    red_left, red_right, green_left, green_right = scan_camera(image, cam_width, cam_height)
    total_red   = red_left + red_right
    total_green = green_left + green_right

    # Tick cooldown down every step
    if goal_cooldown > 0:
        goal_cooldown -= 1

    debug_counter += 1
    if debug_counter >= DEBUG_EVERY:
        debug_counter = 0
        print(f"📷 Red:{total_red} Green:{total_green} | "
              f"Sonar L:{dist_left:.2f} R:{dist_right:.2f} | "
              f"Goals:{goals_scored} Cooldown:{goal_cooldown}")

    # CELEBRATION 
    if celebrating:
        celebrate_timer += 1
        new_x, new_y = x, y
        if celebrate_timer >= CELEBRATE_STEPS:
            celebrating     = False
            celebrate_timer = 0
            respawn_goal()
            goal_cooldown = GOAL_COOLDOWN_STEPS 
            direction = random.uniform(0, 2 * math.pi)
            forwards.setLoop(True)
            forwards.play()

    else:
        # This will only happen when goal's cooldown has expired
        if distance_to_goal() < GOAL_REACH_DIST and goal_cooldown == 0:
            goals_scored += 1
            print(f"\n🏆🏆🏆 GOAL REACHED! Total goals: {goals_scored} 🏆🏆🏆\n")
            celebrating     = True
            celebrate_timer = 0
            forwards.setLoop(False)
            new_x, new_y = x, y

        else:
            near_wall    = abs(x) > ARENA_LIMIT or abs(y) > ARENA_LIMIT
            both_blocked = dist_left < SONAR_LIMIT and dist_right < SONAR_LIMIT
            one_blocked  = dist_left < SONAR_LIMIT or dist_right < SONAR_LIMIT

            # PRIORITY 1 allowing robot to escape wall or corner (got stuck here man)
            if near_wall or both_blocked or stuck_counter > STUCK_LIMIT:
                direction = (direction + math.pi + random.uniform(-0.5, 0.5)) % (2 * math.pi)
                stuck_counter = 0
                new_x = x - SPEED * 3 * math.cos(direction)
                new_y = y - SPEED * 3 * math.sin(direction)
                forwards.setLoop(True)
                forwards.play()

            # PRIORITY 2 make sure blue robot (main) will turn away from red robot
            elif total_red > RED_THRESHOLD:
                print(f"🔴 Red detected! L:{red_left} R:{red_right} — avoiding")
                stuck_counter += 1
                if red_left > red_right:
                    direction = (direction - 0.4) % (2 * math.pi)
                else:
                    direction = (direction + 0.4) % (2 * math.pi)
                new_x, new_y = x, y

            # PRIORITY 3 if the main robot saw green bally it will run toward it lol not run but go toward it
            elif total_green > GREEN_THRESHOLD and goal_cooldown == 0:
                stuck_counter = 0
                if green_left > green_right:
                    direction = (direction + 0.3) % (2 * math.pi)
                else:
                    direction = (direction - 0.3) % (2 * math.pi)
                new_x = x + SPEED * math.cos(direction)
                new_y = y + SPEED * math.sin(direction)
                forwards.setLoop(True)
                forwards.play()

            # PRIORITY 4 blocking the sonar cause it was causing me issues 
            elif one_blocked:
                stuck_counter += 1
                if dist_left > dist_right:
                    direction = (direction + 0.5) % (2 * math.pi)
                else:
                    direction = (direction - 0.5) % (2 * math.pi)
                new_x, new_y = x, y

            # PRIORITY 5 all clear
            else:
                stuck_counter = 0
                new_x = x + SPEED * math.cos(direction)
                new_y = y + SPEED * math.sin(direction)

            # making sure the main robot doesn't into obstacles and actually avoid it
            if collides_with_obstacle(new_x, new_y):
                print(f"🚧 Obstacle collision! Bouncing away...")
                direction = (direction + math.pi + random.uniform(-0.4, 0.4)) % (2 * math.pi)
                new_x, new_y = x, y
                stuck_counter += 1

    # position
    new_x = max(-ARENA_LIMIT, min(ARENA_LIMIT, new_x))
    new_y = max(-ARENA_LIMIT, min(ARENA_LIMIT, new_y))
    trans_field.setSFVec3f([new_x, new_y, z])
    rot_field.setSFRotation([0, 0, 1, direction])
    self_node.resetPhysics()