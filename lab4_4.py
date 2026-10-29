from utils.brick import Motor, reset_brick, wait_ready_sensors
import time
import math

# ==========================================
# TEST MODE TOGGLE
# Set to True  -> ONLY tests the delivery motor (Motor A).
# Set to False -> Runs full navigation + delivery.
# ==========================================
TEST_DELIVERY_ONLY = False

# Grid and robot setting
TILE = 33
WAYPOINTS = [(2, 2), (1, 2), (0, 0), (1, 0)]

RADIUS = 2.17
TRACK_WIDTH = 14.7
SPEED = -100

# Drop mechanism offset: distance behind the robot's center where the block drops (in cm)
DROP_OFFSET = 2

# Maximum time allowed for a turn or drive, in seconds
TURN_TIMEOUT = 20.0
DRIVE_TIMEOUT = 60.0

leftmotor = Motor("A")
rightmotor = Motor("B")
delivery_motor = Motor("C")  # Motor for cube delivery
wait_ready_sensors()

# Position starts at (0, 0), facing +y.
# Heading is measured clockwise from +y.
x = 0.0
y = 0.0
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()
delivery_motor.reset_encoder()
prev_left_encoder = 0
prev_right_encoder = 0


def update_odometer():
    global x, y, theta, prev_left_encoder, prev_right_encoder

    current_left_encoder = -leftmotor.get_encoder()
    current_right_encoder = -rightmotor.get_encoder()

    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    dl = (delta_left / 360.0) * 2 * math.pi * RADIUS
    dr = (delta_right / 360.0) * 2 * math.pi * RADIUS

    # Change in heading, in radians. Positive means clockwise.
    delta_theta = (dl - dr) / TRACK_WIDTH
    delta_distance = (dl + dr) / 2.0

    midpoint_heading = math.radians(theta) + delta_theta / 2.0
    x += delta_distance * math.sin(midpoint_heading)
    y += delta_distance * math.cos(midpoint_heading)
    theta += math.degrees(delta_theta)

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder


def stop():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def deliver_cube():
    """Rotate Motor A to test or execute the delivery mechanism."""
    print("Executing delivery mechanism (Motor A)...")
    
    # Position-based control (rotates 360 degrees)
    #target_pos = delivery_motor.get_encoder() + 360
    #delivery_motor.set_position(target_pos)
    
    # NOTE: If your conveyor belt runs on speed/time instead of position, 
    # comment out the two lines above and uncomment the lines below:
    delivery_motor.set_dps(-40)
    time.sleep(1.4)
    delivery_motor.set_dps(0)

    time.sleep(1.5)
    print("Delivery action finished!")


def shortest_angle(target_heading, current_heading):
    """Return the shortest signed turn from current heading to target."""
    return (target_heading - current_heading + 180) % 360 - 180


def turn_to(target_heading):
    """Turn to an absolute heading using the shortest rotation."""
    target_heading %= 360
    start_time = time.time()

    while True:
        update_odometer()
        remaining = shortest_angle(target_heading, theta)

        if abs(remaining) <= 2:
            break

        # Positive heading change is clockwise.
        if remaining > 0:
            leftmotor.set_dps(SPEED / 2)
            rightmotor.set_dps(-SPEED / 2)
        else:
            leftmotor.set_dps(-SPEED / 2)
            rightmotor.set_dps(SPEED / 2)

        if time.time() - start_time > TURN_TIMEOUT:
            raise TimeoutError("Turn timed out. Check motor directions and odometry signs.")

        time.sleep(0.01)

    stop()


def drive_forward(distance):
    """Drive straight for the requested distance in cm."""
    if distance <= 0:
        return

    start_x, start_y = x, y
    start_time = time.time()

    leftmotor.set_dps(SPEED)
    rightmotor.set_dps(SPEED)

    try:
        while math.hypot(x - start_x, y - start_y) < distance:
            update_odometer()
            print(
                f"X: {x:.2f} cm | Y: {y:.2f} cm | "
                f"Theta: {theta % 360:.1f}°",
                end="\r"
            )

            if time.time() - start_time > DRIVE_TIMEOUT:
                raise TimeoutError("Drive timed out. Check motor directions and odometry signs.")

            time.sleep(0.01)
    finally:
        stop()


def travel_shortest(waypoint):
    """Drive directly to a waypoint using the shortest straight-line path, 
    accounting for the drop mechanism being 2 cm behind the robot's center.
    """
    target_x = waypoint[0] * TILE
    target_y = waypoint[1] * TILE

    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)

    if distance < 1:
        return

    # Heading is clockwise from +y, so atan2(dx, dy) gives the target heading.
    target_heading = math.degrees(math.atan2(dx, dy)) % 360
    turn_to(target_heading)
    
    # Overshoot the waypoint by DROP_OFFSET so that the dropping point 
    # lands precisely on the target waypoint coordinates.
    adjusted_distance = distance + DROP_OFFSET
    drive_forward(adjusted_distance)


if __name__ == "__main__":
    try:
        if TEST_DELIVERY_ONLY:
            print("--- RUNNING IN TEST MODE: Delivery Motor Only ---")
            deliver_cube()
            print("--- Test Complete ---")
        else:
            print("--- RUNNING FULL NAVIGATION & DELIVERY ---")
            for waypoint in WAYPOINTS:
                print(f"\nTravelling to waypoint: {waypoint}")
                travel_shortest(waypoint)
                
                # Trigger delivery mechanism once waypoint is reached
                deliver_cube()
                time.sleep(2)

            print("\nAll waypoints reached and cubes delivered!")
            
    except BaseException as e:
        print(f"\nError: {e!r}")
    finally:
        stop()
        reset_brick()