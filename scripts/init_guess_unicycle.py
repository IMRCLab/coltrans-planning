import numpy as np
import yaml
import argparse

def compute_actions(states):
    """
    Compute (v, w) for each robot based on consecutive states.
    The state space is assumed to be:
    [px, py, alpha1, alpha2, ..., alphaN, theta1, theta2, ..., thetaN-1].
    """
    actions = []
    dt = 1  # Assume a time step of 1 if not specified
    num_robots = len(states[0][2:]) // 2 + 1  # Infer the number of robots based on alpha and theta counts
    for i in range(len(states) - 1):
        curr_state = np.array(states[i])
        next_state = np.array(states[i + 1])

        action = []
        px_current, py_current = curr_state[0], curr_state[1]
        px_next, py_next = next_state[0], next_state[1]

        for r in range(num_robots):
            # Extract alpha for the current and next states
            alpha_current = curr_state[2 + r]
            alpha_next = next_state[2 + r]

            # Compute linear velocity v
            if r == 0:  # For the first robot
                v = np.sqrt((px_next - px_current)**2 + (py_next - py_current)**2) / dt
            else:  # For subsequent robots
                theta_idx = 2 + num_robots + r - 1  # Index of theta in the state
                theta_current = curr_state[theta_idx]
                theta_next = next_state[theta_idx]
                # Compute position of robot r
                px_robot_current = px_current + 0.5 * np.cos(theta_current)
                py_robot_current = py_current + 0.5 * np.sin(theta_current)

                px_robot_next = px_next + 0.5 * np.cos(theta_next)
                py_robot_next = py_next + 0.5 * np.sin(theta_next)

                v = np.sqrt((px_robot_next - px_robot_current)**2 + (py_robot_next - py_robot_current)**2) / dt

            # Compute angular velocity w
            w = (alpha_next - alpha_current) / dt

            # Append (v, w) to actions for this robot
            action.extend([v, w])
            # action.extend([0, 0])
        actions.append(action)

    return np.array(actions)
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", type=str, help="Input YAML file for the table")
    parser.add_argument("--out", default=None, type=str, help="Output YAML file for the table")
    parser.add_argument("--envName", default=None, type=str, help="Environment name")
    parser.add_argument("-w", "--write", action="store_true", help="Write output YAML file")
    args = parser.parse_args()

    with open(args.inp, "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)

    # Extract states
    states = data["result"][0]["states"]
    if not states:
        print("No states found in the input file.")
        return

    # Compute actions
    actions = compute_actions(states)

    # Add actions to the data
    data["result"][0]["actions"] = actions.tolist()

    if args.write:
        with open(args.out, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=None)
        print(f"Updated data with actions written to {args.out}")
    else:
        print(yaml.safe_dump(data, default_flow_style=None))

if __name__ == "__main__":
    main()
