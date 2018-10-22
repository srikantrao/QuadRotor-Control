# 3D Control of a Quadrotor #

## The Tasks ##

For this project, you will be building a controller in C++.  You will be implementing and tuning this controller in several steps.

You may find it helpful to consult the [Python controller code](https://github.com/udacity/FCND-Controls/blob/solution/controller.py) as a reference when you build out this controller in C++.

#### Notes on Parameter Tuning
1. **Comparison to Python**: Note that the vehicle you'll be controlling in this portion of the project has different parameters than the vehicle that's controlled by the Python code linked to above. **The tuning parameters that work for the Python controller will not work for this controller**

2. **Parameter Ranges**: You can find the vehicle's control parameters in a file called `QuadControlParams.txt`. The default values for these parameters are all too small by a factor of somewhere between about 2X and 4X. So if a parameter has a starting value of 12, it will likely have a value somewhere between 24 and 48 once it's properly tuned.

3. **Parameter Ratios**: In this [one-page document](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) you can find a derivation of the ratio of velocity proportional gain to position proportional gain for a critically damped double integrator system. The ratio of `kpV / kpP` should be 4.

### Hover ###

The following code implements the equations of motion described earlier in the 3D control lesson.

<p align="center">
<img src="images/step1_eqn1.png" width="100"/>
</p>

where `c_bar`, `p_bar`, `q_bar`, `r_bar` are represented by

<p align="center">
<img src="images/step1_eqn2.png" width="100"/>
</p>


The quadrotor is able to hover successfully and passes the specification.

```console
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```
<p align="center">
<img src="images/step1_hover.png" width="400"/>
</p>

### Body Rate Control ###

The body rate controller  gets `commanded` and `actual` angular velocities in Body Frame `p_c`, `q_c` and `r_c` as input and generates as output the Moments about the 3 axes.

Since a proportional controller is used, the code to perform this operation is provided below -

```cpp
  V3F I;
  I.x = Ixx;
  I.y = Iyy;
  I.z = Izz;
  momentCmd = I * kpPQR * (pqrCmd - pqr);
```
A value of `~4x` of the default specified was needed for the KpPQR for the Roll and Pitch axes for it to pass the specification.

### Roll/Pitch Control ###

This controller gets the commanded thrust, and the current Euler angles in the Inertial frame and puts out the commanded Roll and Pitch angular velocities in the Body Frame.

The equations to convert the commanded thrust values from Interial Frame to Body Frame are done using the Rotational Matrix using the equations shown below -

<p align="center">
<img src="images/step2_roll_pitch.png" width="400"/>
</p>

These equations are represented in the code below in the function
`RollPitchControl()`.

```cpp
float c = collThrustCmd / mass;
	float b_x_cmd, b_y_cmd = 0.0;

	if (collThrustCmd > 0) {
		 b_x_cmd = -CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
		 b_y_cmd = -CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
	}

	float b_x_err = b_x_cmd - R(0, 2);
	float b_x_p_term = kpBank * b_x_err;

	float b_y_err = b_y_cmd - R(1, 2);
	float b_y_p_term = kpBank * b_y_err;

	pqrCmd.x = (R(1, 0) * b_x_p_term - R(0, 0) * b_y_p_term) / R(2, 2);
	pqrCmd.y = (R(1, 1) * b_x_p_term - R(0, 1) * b_y_p_term) / R(2, 2);

```

The output of the attitude control scenario works as expected after Body Rate Control and Roll/Pitch Control are implemented.

```console
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

### Position Control ###

Position control consists of two components
- Altitude Control implemented in the function `AltitudeControl()`
- XY Control implemented in the function `LateralPositionControl()`

Since all the parameters here are in the Inertial frame, the implementation of the PID controller for altitude is fairly straightforward and is shown below -

```cpp
float b_z = R(2, 2);
	float z_err = posZCmd - posZ;
	float pos_term = kpPosZ * z_err;
	integratedAltitudeError += z_err * dt;

	velZCmd = CONSTRAIN(velZCmd + pos_term, -maxAscentRate, maxDescentRate);  // Keep NED frame directions in mind

	thrust = -(kpVelZ * (velZCmd - velZ) + KiPosZ * integratedAltitudeError + accelZCmd - CONST_GRAVITY) * mass / b_z ;
```
Similarly the XY control is using a PD controller whose code is provided below -

```cpp
```
Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
 - implement the code in the function `AltitudeControl()`
 - tune parameters `kpPosZ` and `kpPosZ`
 - tune parameters `kpVelXY` and `kpVelZ`

If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.

 - implement the code in the function `YawControl()`
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`

Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom.  This is why you often see quadcopters with tilted motors, better yaw authority!

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>


### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

With the two different trajectories, your drones' motions should look like this:

<p align="center">
<img src="animations/scenario5.gif" width="500"/>
</p>


### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
