import sim
import time

from robot_p3dx import RobotP3DX
from navigation import Navigation


if __name__ == '__main__':
    # Connect to CoppeliaSim
    sim.simxFinish(-1)  # Close all pending connections
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    if client_id == -1:
        raise ConnectionError(
            'Could not connect to CoppeliaSim.'
            ' Make sure the application is open.')

    # Start simulation
    sim.simxSynchronous(client_id, True)
    sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)

    # Execute a simulation step to get initial sensor readings
    sim.simxSynchronousTrigger(client_id)
    # Make sure the simulation step has finished
    sim.simxGetPingTime(client_id)

    # Write initialization code here
    dt = 0.05
    steps = 0
    robot = RobotP3DX(client_id, dt)
    navigation = Navigation(dt)
    start_time = time.perf_counter()

    try:
        while True:
            # Write your control algorithm here
            z_us, z_v, z_w = robot.sense()
            v, w = navigation.explore(z_us, z_v, z_w)
            robot.move(v, w)

            # Execute the next simulation step
            sim.simxSynchronousTrigger(client_id)
            # Make sure the simulation step has finished
            sim.simxGetPingTime(client_id)
            steps += 1

    except KeyboardInterrupt:
        # Press Ctrl+C to break the infinite loop and gracefully
        # stop the simulation
        pass

    # Display time statistics
    execution_time = time.perf_counter() - start_time
    print('\n')
    print('Simulated steps: {0:d}'.format(steps))
    print('Simulated time:  {0:.3f} s'.format(steps * dt))
    print('Execution time:  {0:.3f} s ({1:.3f} s/step)'.format(
        execution_time, execution_time / steps))
    print('')

    # Stop the simulation and close the connection
    sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
    # Make sure the stop simulation command had time to arrive
    sim.simxGetPingTime(client_id)
    sim.simxFinish(client_id)
