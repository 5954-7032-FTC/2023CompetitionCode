package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class MecanumDriveByGyro extends MecanumDrive {

    int [] FORWARD_VALUES, REVERSE_VALUES, LEFT_VALUES, RIGHT_VALUES;

    double headingOffset;
    double robotHeading;
    double driveSpeed;
    double turnSpeed;
    double targetHeading;
    double headingError;
    Telemetry.Item T_RF,T_RR,T_LR,T_LF;


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

    public MecanumDriveByGyro(Parameters parameters) {
        super(parameters);
        initAutoMecanum();

        int [] encoders = readEncoders();
        T_RF = _telemetry.addData("RF:", encoders[0]);
        T_RR = _telemetry.addData("RR:", encoders[1]);
        T_LR = _telemetry.addData("LR:", encoders[2]);
        T_LF = _telemetry.addData("LF:", encoders[3]);
        _telemetry.update();

        // start the position update thread:
        imuUpdater = new IMUUpdater();
        new Thread(imuUpdater).start();


        //motor directions RF, RR, LR, LF, rotate
        FORWARD_VALUES = new int[]{ 1, 1, 1, 1,1};
        REVERSE_VALUES = new int[]{-1, -1, -1, -1,-1};
        LEFT_VALUES = new int[]{1,-1,1,-1,1};
        RIGHT_VALUES = new int[]{-1,1,-1,1,1};

    }


    Position imu_pos;

    public void driveForward(double distance) {
        driveRobot(distance, FORWARD_VALUES);
    }

    public void driveLeft(double distance) {
        driveRobot(distance, LEFT_VALUES);
    }

    public void driveReverse(double distance) {
        driveRobot(distance, REVERSE_VALUES);
    }

    public void driveRight(double distance) {
        driveRobot(distance, RIGHT_VALUES);
    }

    int logid=0;
    public void driveRobot(double inches, int [] direction) {
        // Determine new target position, and pass to motor controller
        int moveCounts = moveCounts(inches);
        setTargetPositions(moveCounts, direction);

        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop

        moveRobotDirection(DRIVE_SPEED, 0,direction);
        _telemetry.addData("l"+logid, 0);
        _telemetry.update();

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = direction[4] * getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

            // Apply the turning correction to the current driving speed.
            moveRobotDirection(driveSpeed, turnSpeed, direction);
            _telemetry.addData("l"+logid, "%2.2f %2.2f", driveSpeed, turnSpeed);
            _telemetry.update();

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotDirection(0, 0, FORWARD_VALUES);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
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
            moveRobotDirection(0, turnSpeed,FORWARD_VALUES);
        }
        // Stop all motion;
        moveRobotDirection(0, 0,FORWARD_VALUES);
    }

    public boolean leftIsBusy() {  // return true if either is busy.
        return (motors[1].isBusy() || motors[3].isBusy());
    }

    public int moveCounts(double distance) {
        return (int)(distance * COUNTS_PER_INCH_FORWARD);
    }

    public void moveRobotDirection(double power, double rotate, int [] direction) {
        driveSpeed = power;
        turnSpeed = rotate;
        rotate *= direction[4];
        double [] wheelSpeeds = {
                direction[0]*power - rotate,   // Front Right
                direction[1]*power - rotate, // Rear Rights
                direction[2]*power + rotate,   // Rear Left
                direction[3]*power + rotate  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        int [] encoders = readEncoders();
        T_RF.setValue(encoders[0]);
        T_RR.setValue(encoders[1]);
        T_LR.setValue(encoders[2]);
        T_LF.setValue(encoders[3]);
        _telemetry.update();
        T_angle.setValue(getRobotHeading());
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public boolean rightIsBusy() {
        return (motors[0].isBusy() || motors[2].isBusy());
    }

    public void setRobotCentric(boolean robotCentric) {
        // don't let it change, always robot centric movement.
    }

    public void setTargetPositions(int target, int [] directions) {
        for (int i=0; i<_ENCODER_WHEELS.length; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + target * directions[i]);
        }
    }

    public void stopImu() {
        imuUpdater.StopThis();
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
            moveRobotDirection(0, turnSpeed, FORWARD_VALUES);
        }

        // Stop all motion;
        moveRobotDirection(0, 0, FORWARD_VALUES);
    }

    public void turnRobot(double direction, double holdtime) {
        turnToHeading(TURN_SPEED, direction);
    }

    public synchronized void updateIMUPosition() {
        this.imu_pos = imu.getPosition();
    }






}
