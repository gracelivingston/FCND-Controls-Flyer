# Building a Controller

## Body Rate Control

The Body Rate Control is in `QuadControl::BodyRateControl`. The core
code is a Proportional Controller. First it calculates the error
between the desired body rates and the measured body rates (Line 107):

    (pqrCmd - pqr)

Then it takes the dot product of the error and the angle rate gains
(`kpPQR`):

    kpPQR * (pqrCmd - pqr)

The angle rate gains are much higher horizontally than vertically.
Next it multiplies the gain-adjusted error by the inertial moments of
the thrust arms:

    Ixx * p_d.x
    Iyy * p_d.y
    Izz * p_d.z

And then adds the orthogonal components of the global command:

    (Izz - Iyy) * pqr.z *  pqr.y;
    (Ixx - Izz) * pqr.x * pqr.z;
    (Iyy - Ixx) *  pqr.y * pqr.x;

Both of these terms added together comprise the new body rate command.

## Roll Pitch Control

First we get the constrained $\{x, y\}$ commands (`bx_c`, `by_c`).
They’re determined by finding the required force by multiplying the
command by the mass, and bounding it by the minimum thrusts and
maximum tilt angles:

    float bx_c =
        -CONSTRAIN(accelCmd.x * mass /
                   max(minMotorThrust, collThrustCmd),
                   -maxTiltAngle,
                   maxTiltAngle);
    float by_c =
        -CONSTRAIN(accelCmd.y * mass /
                   max(minMotorThrust, collThrustCmd),
                   -maxTiltAngle,
                   maxTiltAngle);

Then it multiplies the components of the commands by the bank gain
(`kpBank`):

    float bx_c_d = kpBank * (bx_c - bx_a);
    float by_c_d = kpBank * (by_c - by_a);

Finally it rotates the global commands to the rotational frame of
reference:

    pqrCmd.x = (R21 * bx_c_d - R11 * by_c_d) / R33;
    pqrCmd.y = (R22 * bx_c_d - R12 * by_c_d) / R33;
    pqrCmd.z = 0.0f;

## Altitude Controller

This is a full _PID_ controller. It integrates the error in altitude:

    float dp = posZCmd - posZ;
    integratedAltitudeError += dp * dt;

Then it adds the current position error to the vertical command:

    velZCmd += kpPosZ * dp; // p + d


It adjusts the vertical speed by the new command:

    float dv =
      CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate) - velZ;

And finally it translates this to a force in the body reference frame
and compensates for gravity:

    thrust = mass * (9.81f - u) / R(2, 2);

## Implement lateral position

First we amplify the lateral velocity error by the lateral gain and
bound it by the maximum lateral velocity:

    velCmd += kpPosXY * (posCmd - pos);
    if (velCmd.mag() > maxSpeedXY)
    {
        velCmd *= maxSpeedXY / velCmd.mag();
    }

Then we do the same for acceleration:

    accelCmd += kpVelXY * (velCmd - vel);
    if (accelCmd.mag() > maxAccelXY)
    {
        accelCmd *= maxAccelXY / accelCmd.mag();
    }

## Implement yaw control

This calculates the yaw command by scaling the yaw error by the
yaw gain.

## Successful Flight

All of the test scenarios pass:

### Scenario 1

<p align="center">
<img src="scen1.png" width="500"/>
</p>

### Scenario 2

<p align="center">
<img src="scen2.png" width="500"/>
</p>

### Scenario 3

<p align="center">
<img src="scen3.png" width="500"/>
</p>

### Scenario 4

<p align="center">
<img src="scen4.png" width="500"/>
</p>

### Scenario 5

<p align="center">
<img src="scen5.png" width="500"/>
</p>
