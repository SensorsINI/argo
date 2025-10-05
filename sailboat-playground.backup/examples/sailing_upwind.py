import numpy as np
from sailboat_playground.engine import Manager
from sailboat_playground.visualization import Viewer
from sailboat_playground.engine.utils import compute_angle


def wrap(angle):
    while angle < 0:
        angle += 360
    while angle > 360:
        angle -= 360
    return angle


def get_rudder_angle(current_heading, target_heading):
    diff = current_heading - target_heading
    return diff * 0.25


def get_sail_angle(wind_direction, is_tacking):
    if wind_direction > 170 and wind_direction < 190:
        return 2 * (wind_direction - 180)
    elif wind_direction >= 190:
        return 20
    else:
        return -20


def get_target_angle(boat_position, target_position):
    return compute_angle(target_position - boat_position) * 180 / np.pi


if __name__ == "__main__":
    print("**** Sailboat Playground example: sailing_upwind.py")
    print("- Generating simulation data...", end="")
    state_list = []
    stop = False
    steps = 0
    m = Manager(
        "boats/sample_boat.json",
        "environments/playground.json",
        boat_heading=140,
        boat_position=np.array([0, -300]),
        debug=False
    )
    target_position = np.array([0, 400])
    target_heading = 140
    prev_target_heading = 140
    while not stop and steps < 3000:
        state = m.agent_state
        if state["position"][1] <= -410:
            stop = True

        # Check if must tack
        target_angle = get_target_angle(state["position"], target_position)
        if (target_angle >= 140 and prev_target_heading == 40) or (target_angle <= 40 and prev_target_heading == 140):
            is_tacking = True
        else:
            is_tacking = False
            prev_target_heading = target_heading

        # Set target heading according to tacking
        target_distance = np.linalg.norm(state["position"] - target_position)
        if target_distance < 20:
            is_tacking = False
            target_heading = target_angle
        if is_tacking:
            if prev_target_heading == 40:
                target_heading = 140
            else:
                target_heading = 40
        rudder_angle = get_rudder_angle(state["heading"], target_heading)

        # Set sail angle
        sail_angle = get_sail_angle(
            state["wind_direction"], is_tacking)
        m.step([sail_angle, rudder_angle])

        # Check stop condition
        if state["position"][1] > 410:
            stop = True

        state = m.state
        state_list.append(state)
        steps += 1

    print(" done.")
    print("- Showing simulation...", end="")
    buoys = [
        (-150, 400),
        (150, 400),
    ]
    v = Viewer(buoy_list=buoys, map_size=1000)
    v.run(state_list=state_list, simulation_speed=100)
    print(" done.")
