package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.R;

public class MecanumDriveByGyro extends MecanumDrive {
    private final int _LEFT_MOTOR;
    private final int _RIGHT_MOTOR;
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
    public MecanumDriveByGyro(Parameters parameters, int LEFT_MOTOR, int RIGHT_MOTOR) {
        super(parameters);
        this._LEFT_MOTOR = LEFT_MOTOR;
        this._RIGHT_MOTOR = RIGHT_MOTOR;
        this._ENCODER_WHEELS = new int[]{RIGHT_MOTOR,LEFT_MOTOR};
        initAutoMecanum();


        // start the position update thread:
        imuUpdater = new IMUUpdater();
        new Thread(imuUpdater).start();
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


    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
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


    public void moveRobot(double forward,double rotate) {
        driveSpeed = forward;
        turnSpeed = rotate;
        this.moveRect(forward,0,rotate);
    }

    public void moveRobot(double forward, double lateral, double rotate) {
        driveSpeed = forward;
        turnSpeed = rotate;
        lateralSpeed = lateral;
        this.moveRect(forward,lateral,rotate);
    }

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
            moveRobot(0, turnSpeed);
        }
        // Stop all motion;
        moveRobot(0, 0);
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
            moveRobot(0, turnSpeed);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void turnRobot(double direction, double holdtime) {
        turnToHeading(TURN_SPEED, direction);
        holdHeading(TURN_SPEED,direction,holdtime);
    }
    public void driveRobot(double distance, double heading) {
        driveStraight(DRIVE_SPEED,distance,heading);
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(distance * COUNTS_PER_INCH_FORWARD);
        leftTarget = leftGetEncoder() + moveCounts;
        rightTarget = rightGetEncoder() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftSetTargetPosition(leftTarget);
        rightSetTargetPosition(rightTarget);

        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);

    }






    public void setMotorSpeeds(double [] speeds) {
        for (int i=0; i< motors.length; i++) motors[i].setPower(Range.clip(speeds[i], -1, 1));
    }




    public void driveStraight(double distance, double heading) {
        driveRobot(DRIVE_SPEED,distance,heading);
    }
    public void driveLeft(double distance, double heading) {
        int moveCounts = moveCounts(distance);
        leftTarget = leftGetEncoder() -moveCounts;
        rightTarget = rightGetEncoder() + moveCounts;
        driveIt(DRIVE_SPEED,distance,heading);
    }
    public void driveRight(double distance, double heading) {
        int moveCounts = moveCounts(distance);
        leftTarget = leftGetEncoder() -moveCounts;
        rightTarget = rightGetEncoder() + moveCounts;
        driveIt(DRIVE_SPEED,distance,heading);
    }

    public int moveCounts(double distance) {
        return (int)(distance * COUNTS_PER_INCH_FORWARD);
    }


    public void driveRobot(double maxDriveSpeed,
                           double distance,
                           double heading) {

        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH_FORWARD);
        leftTarget = leftGetEncoder() + moveCounts;
        rightTarget = rightGetEncoder() + moveCounts;
        driveIt(maxDriveSpeed,distance,heading);
    }

    public void driveIt(double maxDriveSpeed,double distance, double heading) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftSetTargetPosition(leftTarget);
        rightSetTargetPosition(rightTarget);

        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);

    }




    public void slideXY(double Xinches, double Yinches) {
        double curentHeading = getRobotHeading();
        // measure relative position from where we are now to where we want to be....
        double relX = (imu_pos.x*_INCHES_PER_METER - Xinches);
        double relY = (imu_pos.y*_INCHES_PER_METER - Yinches);
        double angle = Math.atan2(relX,relY);
        angle -= PI_OVER4;
        angle -= getRobotHeading();

        double distance = Math.hypot(relX,relY);

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(distance * COUNTS_PER_INCH_FORWARD);
        double sine  = Math.sin(angle);
        double cosine = Math.cos(angle);
        double scale = ( distance > 1 ) ? DRIVE_SPEED /distance : DRIVE_SPEED /SQRT2 ;

        double [] wheelSpeeds = {
                scale * (distance * sine),   // Front Right
                scale * (distance * cosine), // Rear Rights
                scale * (distance * sine),   // Rear Left
                scale * (distance * cosine)  // Front Left
        };

        rightSetTargetPosition((int)Math.floor(sine*moveCounts));
        leftSetTargetPosition((int)Math.floor(cosine*moveCounts));
        setMotorSpeeds(wheelSpeeds);

        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(curentHeading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        setRunMode(new int[]{_LEFT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(new int[]{_RIGHT_MOTOR},DcMotor.RunMode.RUN_USING_ENCODER);



    }

}
