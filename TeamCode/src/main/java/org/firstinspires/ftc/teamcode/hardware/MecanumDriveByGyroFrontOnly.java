package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import androidx.annotation.NonNull;

public class MecanumDriveByGyroFrontOnly extends MecanumDrive {
    private final int _LEFT_MOTOR;
    private final int _RIGHT_MOTOR;


    double [] FORWARD_VALUES, REVERSE_VALUES, LEFT_VALUES, RIGHT_VALUES;

    class IMUUpdater implements Runnable {
        boolean stopThis = false;

        public void StopThis() {
            this.stopThis = true;
        }

        public void run() {
            while (!stopThis) {
                updateIMUPosition();
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ignored) {}
            }
        }
    }
    IMUUpdater imuUpdater;
    public MecanumDriveByGyroFrontOnly(Parameters parameters, int LEFT_MOTOR, int RIGHT_MOTOR) {
        super(parameters);
        this._LEFT_MOTOR = LEFT_MOTOR;
        this._RIGHT_MOTOR = RIGHT_MOTOR;
        this._ENCODER_WHEELS = new int[]{RIGHT_MOTOR,LEFT_MOTOR};
        initAutoMecanum();


        // start the position update thread:
        imuUpdater = new IMUUpdater();
        new Thread(imuUpdater).start();

        FORWARD_VALUES = new double[]{ 1, 1, 1, 1};
        REVERSE_VALUES = new double[]{-1, -1, -1, -1};
        LEFT_VALUES = new double[]{1,-1,1,-1};
        RIGHT_VALUES = new double[]{-1,1,-1,1};

    }

    public void stopImu() {
        imuUpdater.StopThis();
    }


    Position imu_pos;

    public synchronized void updateIMUPosition() {
        this.imu_pos = imu.getPosition();
    }

    @Override
    public void setRobotCentric(boolean robotCentric) {
        // don't let it change, always robot centric movement.
    }

    public void leftSetTargetPosition(int leftTarget) {
        motors[_LEFT_MOTOR].setTargetPosition(leftTarget);
    }

    public void rightSetTargetPosition(int rightTarget) {
        motors[_RIGHT_MOTOR].setTargetPosition(rightTarget);
    }

    public boolean leftIsBusy() {
        return motors[_LEFT_MOTOR].isBusy();
    }
    public boolean rightIsBusy() {
        return motors[_RIGHT_MOTOR].isBusy();
    }

    public int leftGetEncoder() {
        return motors[_LEFT_MOTOR].getCurrentPosition();
    }
    public int rightGetEncoder() {
        return motors[_RIGHT_MOTOR].getCurrentPosition();
    }



    double headingOffset;
    double robotHeading;
    double driveSpeed;
    double turnSpeed;
    double lateralSpeed;
    double targetHeading;
    double headingError;
    int leftTarget;
    int rightTarget;

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    //move functions are only concerned with wheel speeds.
    public void moveAuto(double power, @NonNull double [] speeds, double rotate) {
        double [] wheelSpeeds = {
                speeds[0]*power - rotate,   // Front Right
                speeds[1]*power - rotate, // Rear Rights
                speeds[2]*power + rotate,   // Rear Left
                speeds[3]*power + rotate  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        T_angle.setValue(getRobotHeading());
    }

    public void moveRobotForward(double power, double rotate) {
        driveSpeed = power;
        turnSpeed = rotate;
        this.moveAuto(power,FORWARD_VALUES,rotate);
    }

    public void moveRobotReverse(double power, double rotate) {
        driveSpeed = power;
        turnSpeed = rotate;
        this.moveAuto(power,REVERSE_VALUES,rotate);

    }

    public void moveRobotLeft(double power, double rotate) {
        turnSpeed = rotate;
        lateralSpeed = power;
        this.moveAuto(power,LEFT_VALUES,rotate);
    }

    public void moveRobotRight(double power, double rotate) {
        turnSpeed = rotate;
        lateralSpeed = power;
        this.moveAuto(power,RIGHT_VALUES,rotate);
    }


    //utility function
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    // drive functions set up to go a certain distance.



    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        // keep looping while we have time remaining.
        while (holdTimer.time() < holdTime) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobotForward(0, turnSpeed);
        }
        // Stop all motion;
        moveRobotForward(0, 0);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (Math.abs(headingError) > HEADING_THRESHOLD) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobotForward(0, turnSpeed);
        }

        // Stop all motion;
        moveRobotForward(0, 0);
    }




    public void turnRobot(double direction, double holdtime) {
        turnToHeading(TURN_SPEED, direction);
        //holdHeading(TURN_SPEED,direction,holdtime);
    }

    public void driveForward(double distance) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH_FORWARD);
        leftTarget = leftGetEncoder() + moveCounts;
        rightTarget = rightGetEncoder() + moveCounts;
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftSetTargetPosition(leftTarget);
        rightSetTargetPosition(rightTarget);

        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveRobotForward(DRIVE_SPEED, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobotForward(driveSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotForward(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveReverse(double distance) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH_FORWARD);
        leftTarget = leftGetEncoder() - moveCounts;
        rightTarget = rightGetEncoder() - moveCounts;
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftSetTargetPosition(leftTarget);
        rightSetTargetPosition(rightTarget);

        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveRobotReverse(DRIVE_SPEED, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobotReverse(driveSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotReverse(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveLeft(double distance) {
        int moveCounts = moveCounts(distance);
        leftTarget = leftGetEncoder() -moveCounts;
        rightTarget = rightGetEncoder() + moveCounts;
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftSetTargetPosition(leftTarget);
        rightSetTargetPosition(rightTarget);
        setRunMode(new int[]{_LEFT_MOTOR,_RIGHT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required front motor driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveRobotLeft(DRIVE_SPEED, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = 0;//getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

            // Apply the turning correction to the current driving speed.
            moveRobotLeft(lateralSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotLeft(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveRight(double distance) {
        int moveCounts = moveCounts(distance);
        leftTarget = leftGetEncoder() + moveCounts;
        rightTarget = rightGetEncoder() - moveCounts;
        leftSetTargetPosition(leftTarget);
        rightSetTargetPosition(rightTarget);
        //Telemetry.Item positions = _telemetry.addData("distance", "%d, %d",(leftTarget-leftGetEncoder()), (rightTarget-rightGetEncoder()));


        setRunMode(new int[]{_LEFT_MOTOR,_RIGHT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required front motor driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveRobotRight(DRIVE_SPEED, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {
            //positions.setValue("%d, %d",(leftTarget-leftGetEncoder()), (rightTarget-rightGetEncoder()));
            //_telemetry.update();
            // Determine required steering to keep on heading
            turnSpeed = 0;//-1.0 * getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

            // Apply the turning correction to the current driving speed.
            moveRobotRight(lateralSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotRight(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int moveCounts(double distance) {
        return (int)(distance * COUNTS_PER_INCH_FORWARD);
    }








}
