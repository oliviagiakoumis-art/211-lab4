from utils.brick import Motor, reset_brick, wait_ready_sensors, EV3ColorSensor
import time
import math

# ADDED: Toggle between testing the delivery mechanism and full operation.
# True = test delivery only; False = navigate and deliver cubes.
TEST_DELIVERY_ONLY = False

# Constants from Lab 2
RADIUS = 2.13
TRACK_WIDTH = 14.5
SPEED = -100
TILE = 32.48
THRESHOLD = 200
STEPS = 3
CORRECTION = 0

WAYPOINTS = [
    [1, 1],
    [2, 2],
    [0, 2],
    [0, 0]
]

leftmotor = Motor("A")
rightmotor = Motor("B")

# ADDED: Initialize the motor responsible for cube delivery.
delivery_motor = Motor("C")

colorsensor = EV3ColorSensor(1)

x = 0.0
y = 0.0
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()

# ADDED: Reset the delivery motor's encoder.
delivery_motor.reset_encoder()

prev_right_encoder = 0
prev_left_encoder = 0

wait_ready_sensors()


def update_odometer():
    # Existing odometry function from Lab 2
    global x, y, theta, prev_right_encoder, prev_left_encoder

    current_left_encoder = -leftmotor.get_encoder()
    current_right_encoder = -rightmotor.get_encoder()

    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    delta_distance_left = (
        delta_left / 360.0
    ) * 2 * math.pi * RADIUS

    delta_distance_right = (
        delta_right / 360.0
    ) * 2 * math.pi * RADIUS

    delta_theta = (
        delta_distance_left - delta_distance_right
    ) / TRACK_WIDTH

    delta_distance = (
        delta_distance_left + delta_distance_right
    ) / 2.0

    # Heading convention: 0 degrees points along +y,
    # increasing clockwise toward +x.
    delta_x = delta_distance * math.sin(
        math.radians(theta) + delta_theta / 2.0
    )
    delta_y = delta_distance * math.cos(
        math.radians(theta) + delta_theta / 2.0
    )

    x += delta_x
    y += delta_y
    theta = (theta + math.degrees(delta_theta)) % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder


# ADDED: Activate the cube delivery mechanism.
def deliver_cube():
    """Run the delivery motor to dispense one foam cube."""
    print("\nActivating cube delivery mechanism...")

    # Run Motor C at -40 degrees per second for 1.4 seconds.
    delivery_motor.set_dps(-40)
    time.sleep(1.4)

    # Stop the delivery motor.
    delivery_motor.set_dps(0)

    # Allow the mechanism to settle before continuing.
    time.sleep(1.5)

    print("Delivery action finished!")


# NEW: Check whether a signed target displacement is complete.
def target_reached(current_position, target_position, distance):
    if distance >= 0:
        return current_position >= target_position
    else:
        return current_position <= target_position


# CHANGED: Supports positive AND negative distances.
def drive_forward(distance, direction):
    if direction == "x":
        start_position = x
    else:
        start_position = y

    target_position = start_position + distance

    while True:
        current_position = x if direction == "x" else y

        if target_reached(current_position, target_position, distance):
            break

        leftmotor.set_dps(SPEED)
        rightmotor.set_dps(SPEED)

        update_odometer()

        print(
            f"X: {x:.2f} cm | Y: {y:.2f} cm | "
            f"Theta: {theta:.2f}°",
            end="\r"
        )

        time.sleep(0.01)

    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


SEARCH_STEP = 0.8       # seconds per initial search step
MAX_SEARCH_STEPS = 50
SENSOR_DELAY = 0.02


def line_found():
    return colorsensor.get_value()[0] < THRESHOLD


def stop_motors():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def steer(direction):
    if direction == "left":
        leftmotor.set_dps(10)
        rightmotor.set_dps(SPEED * 0.25)
    else:
        leftmotor.set_dps(SPEED * 0.25)
        rightmotor.set_dps(10)


def correct_path():
    # Search alternately left and right, expanding the search.
    for step in range(1, MAX_SEARCH_STEPS + 1):
        direction = "right" if step % 2 == 1 else "left"
        duration = SEARCH_STEP * step

        steer(direction)
        elapsed = 0.0

        while elapsed < duration:
            update_odometer()

            if line_found():
                stop_motors()
                return True

            time.sleep(SENSOR_DELAY)
            elapsed += SENSOR_DELAY

    stop_motors()
    return False


def drive_forward_correct(distance, direction):
    if direction == "x":
        start_position = x
    else:
        start_position = y

    target_position = start_position + distance

    while True:
        current_position = x if direction == "x" else y

        if target_reached(
            current_position, target_position, distance
        ):
            break

        if line_found():
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)
        else:
            stop_motors()

            if not correct_path():
                print("\nLine not found. Robot stopped.")
                return False

            # The line was found; resume driving.
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)

        update_odometer()

        print(
            f"X: {x:.2f} cm | Y: {y:.2f} cm | "
            f"Theta: {theta:.2f}°",
            end="\r"
        )

        time.sleep(0.01)

    stop_motors()
    return True


def get_shortest_angle(target_heading, current_heading):
    """Return the shortest signed turn to reach a target heading."""
    return (target_heading - current_heading + 180) % 360 - 180


def execute_turn(target_heading):
    """Turn to a target heading using the shortest direction."""
    target_heading = target_heading % 360.0
    delta = get_shortest_angle(target_heading, theta)

    # Already facing the target within 6 degrees.
    if abs(delta) < 6:
        return

    # Positive delta = clockwise/right
    # Negative delta = counterclockwise/left
    if delta > 0:
        leftmotor.set_dps(SPEED)
        rightmotor.set_dps(-SPEED)
    else:
        leftmotor.set_dps(-SPEED)
        rightmotor.set_dps(SPEED)

    try:
        while abs(get_shortest_angle(target_heading, theta)) > 4:
            update_odometer()
            time.sleep(0.01)
    finally:
        leftmotor.set_dps(0)
        rightmotor.set_dps(0)
        time.sleep(1)


def travel_safe(waypoint):
    dy = waypoint[1] * TILE - y

    if abs(dy) > 6:
        execute_turn(0 if dy > 0 else 180)

        if not drive_forward_correct(dy, "y"):
            return False

    dx = waypoint[0] * TILE - x

    if abs(dx) > 6:
        execute_turn(90 if dx > 0 else 270)

        if not drive_forward_correct(dx, "x"):
            return False

    return True


# ADDED: Main program supports delivery-only testing or full operation.
try:
    if TEST_DELIVERY_ONLY:
        # Test the delivery motor without moving the robot.
        print("--- RUNNING IN TEST MODE: Delivery Motor Only ---")
        deliver_cube()
        print("--- Test Complete ---")

    else:
        print("--- RUNNING FULL NAVIGATION & DELIVERY ---")

        for waypoint in WAYPOINTS:
            print(f"\nTravelling to waypoint: {waypoint}")

            # Navigate to the waypoint.
            if not travel_safe(waypoint):
                print("Navigation stopped because line recovery failed.")
                break

            # ADDED: Deliver one cube after successfully completing navigation.
            print(f"\nWaypoint {waypoint} reached. Delivering cube...")
            deliver_cube()

            # ADDED: Brief pause before travelling to the next waypoint.
            time.sleep(2)

        else:
            # This executes only if all waypoints finish without a break.
            print("\nAll four waypoints reached and cubes delivered!")

finally:
    # Stop both navigation motors.
    stop_motors()

    # ADDED: Ensure the delivery motor is also stopped.
    delivery_motor.set_dps(0)

    reset_brick()
