
# CS 7638: Artificial Intelligence for Robotics

### Quick Instructions
- To run all test cases from command line, use: `python DualRotor_TestSuite.py`
- To run a single test case, use: `python -m unittest DualRotor_TestSuite.Part_1_a_TestCase.test_case01` (replace “test_case01” with the test case number you want to run)

### Project Description
Autonomous drones are used to maintain critical infrastructure, e.g., inspect gas pipelines for leaks. In this project you will implement a PID controller for an autonomous drone to fly to a target elevation and horizontal position and hover at some target location for a specified time.

A PID controller is a component of a feedback control system. The system (usually called “plant”) is supposed to be maintained at a target “steady” state. For example, a car that needs to stay in the middle of a lane, or a thermostat that needs to maintain a particular temperature, etc. The Drone starts on the ground with its rotors off. For simplicity we only consider a two dimensional world, and a dual rotor drone. We also don’t consider any ground effects (https://www.rchelicopterfun.com/ground-effect.html) while simulating the drone.

Consider the simple case first, where the Drone only has to lift off vertically and achieve a target elevation and hover there.

1. **Target elevation**
    - `y1`
    - `(x1=0)`
    - `y-axis`
    - `Starting on ground`
    - `x-axis y=0 x=0`

We have to induce a certain rpm (rotations per minute) equally in both the rotors. As the rpm of its rotors increase, they push the air down and this exerts a net upward force (thrust) on the Drone.

    - `thrust thrust`
    - `↑`
    - `weight`

When this force equals the weight of the Drone, it starts to hover. Increasing the rpm beyond this point will lift the Drone up. To reach a certain elevation within some limited time, the thrust has to be sufficiently high. But as the drone nears the target, we have to start reducing the thrust until it is back equal to its weight ideally right at the target elevation. There, we have to maintain this thrust in order for it to hover for the desired period.

Now let’s consider the situation where we also want to move the drone horizontally, besides vertically.

- **Target position**
    - `x1`
    - `y-axis`
    - `Starting on ground`
    - `y=0`
    - `x-axis x=0`

For this we need to tilt the drone sideways, which is called Roll (measured in angle). This is done by varying the thrust (or rpm) between the rotors of the Drone. In a Dual Rotor Drone, if thrust on the right side is higher, the Drone will roll left. If thrust on the left side is higher, then the Drone will roll right.

    - `2`
    - `1`
    - `Positive Roll`
    - `angle`
    - `X-axis`
    - `x-axis`
    - `Negative roll angle`

When the drone is at rest, it has a roll angle of zero. A positive roll angle corresponds to a leftward tilt, and a negative roll angle corresponds to a rightward tilt.

Once the drone rolls in the appropriate direction, the horizontal component of thrust moves the drone in the horizontal direction. The Drone needs to have enough roll to reach the target `x1` location in a specific timeframe (specified by the test case). Note that the vertical component of thrust in each rotor keeps the drone levitating. Combined, they should equal or exceed the weight of the drone to keep it hovering, or lift it up to the desired height.

As it reaches close to the target `x1`, we need to reduce the roll until it reaches 0 degree, ideally exactly at the target. Then it has to hover there for a specified time. Note that in order to roll and move side-ways, the Drone needs to be at some elevation, so roll is applied along with some thrust.

We only need to return a single thrust and roll angle from the PID controller (actually, a change in thrust and roll angle), and the simulator will take care of inducing the appropriate rpms in the rotors. If the roll angle is zero, the thrust is going to be equally distributed. Otherwise, it will be distributed according to the roll angle.

You will implement the PID controllers for thrust and roll in the file `drone_pid.py`. You will be provided a target elevation for the PID Thrust controller, and a target horizontal position for the PID Roll controller. You will also implement the function “find_parameters” to find and tune the gain values for your controllers. Here you can implement the twiddle algorithm (or any other method if you like). The `find_parameters` method will be passed a “run_callback” function, which you will call passing in your PID gain parameters to simulate the drone. The `run_callback` function is the `run()` function in `DroneSimulator`. It will take in your PID parameters and invoke your PID control functions to move the drone.

As a reminder, the PID control value (alpha) is calculated by the following formula: `alpha = tau_p * error + tau_i * error_integral + tau_d * error_derivative`

### Files
These are some key files of the project that you would need to know about:
1. `DualRotor_TestSuite.py`: This file contains the test cases, and is your starting point to run the simulation. To run a single test case:
    - `python -m unittest DualRotor_TestSuite.Part_1_a_TestCase.test_case01`
To run all test cases:
    - `python DualRotor_TestSuite.py`
You can also change various parameters in the file (e.g. `VISUALIZE`, `DEBUG`, `TWIDDLE`). For example, you might want to turn off `TWIDDLE` to run a test with manually supplied PID gain values to test your code. Each test case in this file do the following logical sequence of work:
    1. Initialize `DroneSimulator`
    2. Call `find_parameters()` in `drone_pid.py` (this will be your implementation of twiddle)
    3. Call `DroneSimulator.run()`, with the PID values found from the step above.
    4. Calculate your score.
2. `DroneSimulator.py`: This file contains the code to run the simulation of the Drone. The `initialize()` method is called by the test suite. The `run()` method executes the following high-level logic:
    1. Do the following in a loop for the specified timesteps:
        i. Call your `pid_thrust()` and `pid_roll()` implementations in `drone_pid.py` file, passing in the target coordinates and the Drone’s current coordinates.
        ii. Call `DualRotor.move()`, passing in the thrust and roll values retrieved from the above step.
    2. Calculate error values used by `DualRotor_TestSuite` to calculate a score.
Each iteration of the loop represents one timestep and is assumed to be 1/10th of a second (the actual frequency of a real hardware-in-a-loop system could vary depending on the actuators involved).The `run()` method will also be passed to your implementation of twiddle in `find_parameters_XXX()` function of `drone_pid.py`. Your twiddle code will call this `run()` method to simulate the drone flight and tune the PID gain values.
3. `drone_pid.py`: This is the only file that you need to put your code in. The descriptions of the functions you need to implement are given below:
    1. `pid_thrust` - In this function, you have to implement the code for a PID controller for Thrust. It is important to note that what you return from this function is the change in thrust, not a target thrust. The function is called from `DroneSimulator.run()` in a loop. At every step, there is a max change in rpm that can be applied by the Drone. This can be a positive or negative change. If the value returned by your implementation is going to cause the rpm to change outside of the max possible change in rpm per timestep, it is clipped at the max (or min if negative).
    2. `pid_roll` - In this function, you have to implement the code for a PID controller for Roll. The return value is the change in roll angle. The function is called in a loop from `DroneSimulator.run()`. The change in roll at each step is also constrained by the max and min change in rpm at each rotor. For the purpose of this assignment, there is a max roll angle that the Drone can’t go beyond, and any change beyond this value will be ignored. As roll can’t be applied without some elevation, this function is always called along with the `pid_thrust` function by the simulator.
    3. `find_parameters_XXX` - In this function, you will implement the twiddle algorithm to find gain values for your controllers. You will be passed a reference to the `DroneSimulator.run()` method, which you will call with the thrust and roll PID gain values that you want to test from your twiddle algorithm. For simplicity, this assignment provides 3 `find_parameter()` functions, one only for thrust, another one for thrust and roll, and a third one for Integral gain (with thrust).

### Submitting your Assignment
Your submission will consist of the `drone_pid.py` file (only) which will be uploaded to `Canvas->GradeScope`. Do not archive (zip,tar,etc) it. Your code must be valid python version 3 code.

We ask that you keep any testing code in a separate file that you do not submit. Your code should also NOT display a GUI or Visualization when we import or call your function under test.

### Calculating your score
The max score for the entire project is 100. There are 8 test cases in total - these are defined in the file `test_cases.txt`. Each test case will have a continuous score upto the maximum points for that test case. Your score is determined by how well the drone meets the requirements of hovering, velocity and oscillations. The test case descriptions and point distributions are as follows:
1. Test cases 1, 2, 3 (30 points each, total 90 points): These test cases only require that your drone reaches a target elevation, i.e., no horizontal movement is required. So it will only test your Thrust PID controller, and your twiddle algorithm’s ability to find good gain values for that.
2. Test case 4 (2 points): This test case tests for elevation (Thrust PID) only, and specifically tests for the Integral part of your controller. It simulates an error in the drone’s rotors so that they lose a certain RPM at every time step. Therefore, an Integral gain will most likely be needed to reach the target. In addition, during the final run of the simulator (for calculating the score), the program simulates an additional force pulling down on the drone for part of the time window. You may also need to handle Integral Wind-up and Control Saturation in this case.
3. Test cases 5, 6, 7, 8 (2 points each, total 8 points): These test cases require both an elevation as well as a horizontal movement. So they will test both your Thrust PID controller, as well as your Roll PID controller. Consequently, your twiddle algorithm will need to find gain values for both Thurst and Roll PID controllers.

To understand what is contained in `test_cases.txt`, here is the breakdown of `testcase_1` as an example:
```json
"testcase_1": {
    "path": [[0,0], [0,5]],
    "target_time": [0, 15],
    "hover_time": [0, 5],
    "max_velocity": 1.5,
    "max_oscillations": 1,
    "score_weight": 0.30
}
```
- `testcase_1`: The name of the test case. This is loaded by the corresponding `test_caseXX()` method in `DualRotor_TestSuite.py`.
- `path`: Defines the target path that the drone needs to follow in (x,y) coordinates. In the above example, the path starts from coordinates (0,0) and go to (0,5) (i.e., straight upward). Note that the drone’s starting position is always (0,0). The path can consist of multiple segments. For simplicity, each segment is either straight vertical or straight horizontal. Vertical can be up or down. Horizontal can be right or left.
- `target_time`: Maximum number of seconds in which the drone should reach the corresponding target coordinates. In the above example, the Drone should reach (0,5) in 15 seconds. (The Drone always starts at (0,0) so the target time to the first target of (0,0) is 0 seconds). Recall from above that each iteration of the loop in `DroneSimulator.run()` is 1/10th of a second. So 15 seconds translates to 150 iterations (time steps) of the loop, and that’s also what you will see in the visualizations.
- `hover_time`: Number of seconds after the `target_time` that the Drone should hover at the target coordinates. In the above example, it doesn’t need to hover at (0,0) (since that’s the starting position), and should hover for 5 seconds at (0,5) from the 16th to the 20th second. If the drone reaches (0,5) before the 16th second, it still should hover at (0,5) till the 20th second (so hover time can be more than 5 seconds, but not less). (Note that like the `target_time`, 5 seconds of hover time also appear as 50 timesteps in the visualizations).
- `max_velocity`: The max velocity in meters/sec that the Drone should have at any point along its path. In the above case, it is set at 1.5 meters/sec. This is another way to control how fast the drone should reach a target location.
- `max_oscillations`: The maximum number of oscillations that your controller is allowed around the target. In the above example, it is 1, which means in flying from (0,0) to (0,5) (which is straight up), it is acceptable if the drone goes slightly above (0,5) before coming back down to it, as long as it happens only once. If after this, it goes down below (0,5), and then comes back up to hover at (0,5), that is counted as a second oscillation which will incur a penalty.
- `score_weight`: This is the weight of the test case out of a total of 100 (this should match the above scoring description).

You will also see a few other attributes for `testcase_4`:
- `rpm_error`: Number of RPMs that the drone loses at every time step. This is used for simulating a systemic error in the drone.
- `extra_load`: Mass of the extra load that the program simulates the drone is carrying while running the scoring run of `simulator.run()`. This is not used during twiddle.
- `extra_load_carry_time`: Length of time in seconds that the drone carries the above mentioned extra load.

### Control Saturation and Integral Wind-up
In a feedback control system, control saturation occurs when an actuator (e.g. motor or steering angle) has reached its limit, but the steady state (i.e. the desired target, like target x, y position for drone) has not been reached. So the controller keeps increasing the output of the actuator. This in itself may not be a problem because many real systems will simply ignore the additional commands. However the problem occurs if there is an integral term in the controller. It will continue to accumulate more and more error (called Integral windup), as it is not able to release any of this error because the actuator is operating at its limit. When the system reaches the steady state, the error for P term is zero which will try to lower the actuator output. However, now the Integral term begins to unwind and it will continue to move the actuator in the same direction and take the system beyond the steady state. To handle this situation, you need to check if the control output is saturated and handle the integral term appropriately. In this assignment, we pass you a flag to indicate if control is saturated, but you will need to check for it in your code and handle the integral term appropriately.

### Testing Your Code
NOTE: The test cases in this project are subject to change.

We have provided a testing suite similar to the one we’ll be using for grading the project, which you can use to ensure your code is working correctly.

We are using the `Canvas > GradeScope` antograder system which allows you to upload and grade your assignment with a remote / online autograder. You must submit your `drone_pid.py` file to `Gradescope` to receive credit.

We are likely, but not required, to use the last grade you receive, (or your selected grade) via the `Canvas->GradeScope` autograder as your final grade at our discretion. (See the “Online Grading” section of the Syllabus.)

### Academic Integrity
You must write the code for this project alone. While you may make limited usage of outside resources, keep in mind that you must cite any such resources you use in your work (for example, you should use comments to denote a snippet of code obtained from StackOverflow, lecture videos, etc).

You must not use anybody else’s code for this project in your work. We will use code-similarity detection software to identify suspicious code, and we will refer any potential incidents to the Office of Student Integrity for investigation. Moreover, you must not post your work on a publicly accessible repository; this could also result in an Honor Code violation [if another student turns in your code]. (Consider using the GT provided Github repository or a repo such as Bitbucket that doesn’t default to public sharing.)

```python

def pid_thrust(target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
    Student code for Thrust PID control. Drone's starting x, y position is (0, 0).

    Args:
    target_elevation: The target elevation that the drone has to achieve
    drone_elevation: The drone's elevation at the current time step
    tau_p: Proportional gain
    tau_i: Integral gain
    tau_d: Differential gain
    data: Dictionary that you can use to pass values across calls.
        Reserved keys:
            max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.

    Returns:
        Tuple of thrust, data
        thrust - The calculated change in thrust using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.
            Reserved keys:
                max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    '''

    thrust = 0

    return thrust, data


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data:dict() = {}):
    '''
    Student code for PD control for roll. Drone's starting x,y position is 0, 0.

    Args:
    target_x: The target horizontal displacement that the drone has to achieve
    drone_x: The drone's x position at this time step
    tau_p: Proportional gain, supplied by the test suite
    tau_i: Integral gain, supplied by the test suite
    tau_d: Differential gain, supplied by the test suite
    data: Dictionary that you can use to pass values across calls.

    Returns:
        Tuple of roll, data
        roll - The calculated change in roll using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.

    '''

    roll = 0

    return roll, data


def find_parameters_thrust(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test cases only.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune
    params = [0,0,0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = None

    # Implement your code to use twiddle to tune the params and find the best_error

    # Return the dict of gain values that give the best error.

    return thrust_params, roll_params

def find_parameters_with_int(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test case with Integral error

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune, e.g.,
    params = [0,0,0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = None

    # Implement your code to use twiddle to tune the params and find the best_error

    # Return the dict of gain values that give the best error.

    return thrust_params, roll_params

def find_parameters_with_roll(run_callback, tune='both', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you will
    find gain values for Thrust as well as Roll PID controllers.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    # Initialize a list to contain your gain values that you want to tune, e.g.,
    params = [0,0,0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = None

    # Implement your code to use twiddle to tune the params and find the best_error

    # Return the dict of gain values that give the best error.

    return thrust_params, roll_params

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = ''
    return whoami
```


# Tips for Drone Control PID project

1. **First make sure your PID control functions are correct. A good way to do that is to try some manual PID tuning first. This will also help you understand your system better. To do this, you can temporarily set the TWIDDLE flag to False in DualRotor_TestSuite.py and try setting manual values for thrust_params and roll_params in run_test() method. Remember to turn TWIDDLE flag back True after this. A good resource for a manual tuning method is here: https://youtu.be/uXnDwojRb1g**
2. **Make sure that you understand how Twiddle should behave for this project. Some manual tuning as described in tip #1 above will help with that. There is a subtle difference between this project and the example of steering a car in the lectures. In particular, when you steer a car during twiddle, you will affect it immediately even if the steering angle is very small, and you will receive an immediate feedback which you can use to determine the direction of your search. In the case of a drone, it requires a minimum value of Thrust (and hence tau_p) to take off. Before that value is reached, you won’t get any useful feedback because the error is going to stay the same. You need to think a bit about where that might cause a problem in the twiddle implementation from lectures. One thing that will help you is to turn on the VISUALIZE_TWIDDLE flag in DualRotor_TestSuite.py. You can pause at each iterations of twiddle and examine your parameter values (tau), and see if they are moving in the expected direction based on how the error changed (or did not change). For example, if you see that twiddle keeps effectively reducing your thrust even though your drone did not move enough, then that could indicate that you need to check your implementation. You can also turn on DEBUG_TWIDDLE in DualRotor_TestSuite.py. This will print out values on your console that you can inspect.**
3. **Twiddle is susceptible to local minima. This means that based on different starting values of your gain parameters or hyperparameters (tau, dp, dp change factor), twiddle may end up with different ending values. One thing you could do is to try different initial values for your parameters or hyperparameters. But first try stepping through your code as described in tip #2 (with initial from lectures) and make sure your Twiddle implementation is doing what you expect it to do. Once you are comfortable with your implementation’s behavior and you are confident that you just need to expand your search, then you can try the following:**
    - **Grid Search: Basically put your twiddle loop within another loop that tries different initial values.**
    - **Manual tuning. You can temporarily set the TWIDDLE flag to False in DualRotor_TestSuite.py and try setting manual values for thrust_params and roll_params in run_test() method. Once you arrive at some relatively good values manually, you can turn TWIDDLE flag back on and use those values as initial values in your find_parameters_XXX() function in drone_pid.py. A good resource for manual tuning method is here: https://youtu.be/uXnDwojRb1g**
4. **For find_parameters_with_int(), where you also have to find an Integral gain, you can also try Grid Search (see 3a) with a few different integral values. Or you can manually come up with an initial Integral value and let Twiddle fine tune it.**
5. **For find_parameters_with_roll(), you can try two strategies:**
    - **The first approach is simultaneous tuning. Here you simply extend the Twiddle algorithm to tune 6 parameters instead of 3. You might need to make a choice and experiment with what sequence works best. For example, place all 3 parameters for Thrust first, followed by parameters for Roll. Or place P for Thrust and Roll, followed by D for both, followed by I for both.**
    - **Sequential tuning: Run the Twiddle loop for thrust params first, holding roll params constant. Then hold thrust params constant and run Twiddle for roll. Repeat until error improvement is less than some threshold.**
6. **Stopping criteria: Depending on your implementation, you could find that the simple stopping criteria for Twiddle from lectures (i.e. sum(dp) < tol) may not be enough. This could be indicated by your program running for a very long time even though your error has decreased to an acceptable value, or timing out in gradescope. You may want to use additional stopping criteria if this is the case. Here are a couple of suggestions, although there could be others:**
    - **Keep a track of the best_error across iterations of your loop, and stop if the improvement in the best_error is less than a threshold over some number of iterations.**
    - **Stop if the error is below an acceptable threshold.**
7. **Error: In the lectures, when you twiddle the error that your run() method returned was the total Cross Track Error for the whole path or some form of it. In this project, the run_callback() method returns multiple values. You have to use them to create an error value that you will minimize across iterations of Twiddle. At a minimum you will need to use the hover_error. Sometimes that is enough, but depending on your implementation, you may find that if you just reduce the hover_error, you may exceed the max velocity and/or max oscillations. So you can convert the velocities and oscillations returned from run_callback() into an error measure. For example, consider velocity. The return values contain a maximum velocity your drone should stay within. You are also given the drone’s max velocity at any point in its flight. You can convert this info into an error value. So if you come up with gain values which make the drone exceed the max velocity, your error should be higher than the values that keep it within the max velocity. Furthermore, the error should increase the more you exceed the max allowed velocity. The case of oscillations is similar. For an example of how to convert the returned values into an error, see DualRotor_TestSuite.py > Part_1_a_TestCase > get_score() method.**
**Also, depending on your Twiddle implementation, if you are using all 3 errors (hover, velocity and oscillation), you might notice that your hover_error might want to pull your gains in one direction, but your velocity error is pulling them in the opposite direction. Same could happen between hover and oscillation error, or velocity and oscillation errors. If you are running into this issue, you will notice this especially in the initial iterations of Twiddle. In this case, you may need to use some weighted combination of the 3 errors.**
**Overall, start with a simple strategy, then add to it if needed.**

# CS7638 OMS Drone Control PID

# CS Tutor | 计算机编程辅导 | Code Help | Programming Help

# WeChat: cstutorcs

# Email: tutorcs@163.com

# QQ: 749389476

# 非中介, 直接联系程序员本人
