/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.LiftClawLinear;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaKey;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

import java.util.List;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "LeftAutoBetter")
public class AutoDriveByGyro_Linear_Left extends LinearOpMode {

    /* Declare OpMode members. */

    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;   // 1120 per revolution
    static final double     DRIVE_GEAR_REDUCTION    = 24.0/32.0; //   3/4
    static final double     WHEEL_DIAMETER_INCHES   = 96/25.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 0.5 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.02;     // Larger is more responsive, but also less stable



    static final int LEFT_ENCODER = 3;
    static final int RIGHT_ENCODER = 0;

    LiftClawLinear _liftclaw;
    Servo _arm_release;


    DcMotor [] motors;
    @Override
    public void runOpMode() {

        // set up MovementThread

        motors = new DcMotor[4];
        motors[0]=hardwareMap.dcMotor.get("D_FR");
        motors[1]=hardwareMap.dcMotor.get("D_RR");
        motors[2]=hardwareMap.dcMotor.get("D_RL");
        motors[3]=hardwareMap.dcMotor.get("D_FL");

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        //_move = new LinearMovement(motors,telemetry);


        // setup LiftClawOld
        final DcMotor [] lift_motors = {
                hardwareMap.dcMotor.get("LIFT"),
                hardwareMap.dcMotor.get("LIFT2")
        };
        final Servo[] lift_servos = {
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };
        final TouchSensor bottom_stop = hardwareMap.touchSensor.get("BSTOP");


        final DistanceSensor post_sensor = hardwareMap.get(DistanceSensor.class, "C_STOP");

        _liftclaw = new LiftClawLinear(
                lift_motors,
                lift_servos,
                bottom_stop,
                post_sensor,
                telemetry
        );

        _arm_release =  hardwareMap.servo.get("ARM_RELEASE");

        WebcamName camera =  hardwareMap.get(WebcamName.class, "Webcam 1");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TensorFlowInit(tfodMonitorViewId,telemetry,camera);


        // Initialize the drive system variables.
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftsetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightsetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftsetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightsetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AutoTransitioner.transitionOnStop(this, "TeleOp");
        double maxConfidence=-1;
        String maxLabel = null;

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
/*            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //_T_count.setValue( updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getConfidence() > maxConfidence) {
                        maxLabel = recognition.getLabel();
                        break;
                    }
                }
                //_telemetry.update();
            }
*/
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.addData("Zdeg", angles.firstAngle);
            telemetry.addData("Ydeg", angles.secondAngle);
            telemetry.addData("Zdeg", angles.thirdAngle);
            telemetry.update();

        }

        // Set the encoders for closed loop speed control, and reset the heading.
        leftsetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightsetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        //_liftclaw.Calibrate();

        //is there a location?
        ElapsedTime et = new ElapsedTime();
        //we have 15 secs to look for the cone
        while (et.seconds() < 12 && maxLabel == null ) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getConfidence() > maxConfidence) {
                        maxLabel = recognition.getLabel();
                        maxConfidence = recognition.getConfidence();
                    }
                } // after this loop, the most confident choice will be made
            }
        }

        int _whereToEnd_value;
        if ( maxConfidence <0 ) {
            _whereToEnd_value = (int)Math.floor(Math.random()*100%3)+1; // 10 points on park
        }
        else {
            _whereToEnd_value = maxLabel.getBytes()[0] - '0'; // 3.3 points on average.
        }
        telemetry.addData("Endpoint:",_whereToEnd_value);

        //_liftclaw.reset_zero();
        //_liftclaw.runToPos(900);

        final double NORTH=0;
        final double EAST=-90;
        final double WEST=90;
        final double SOUTH=180;

        driveStraight(DRIVE_SPEED, 19.5, NORTH);    // Drive Forward 18"
        //holdHeading(TURN_SPEED, NORTH, 1);
        _arm_release.setPosition(0.85); // release the arm.
        switch (_whereToEnd_value) {
            case 1:
                turnRobot(EAST,1);
                driveStraight(DRIVE_SPEED, -16, EAST);
                break;
            case 2:
                turnRobot(EAST, 1);
                turnToHeading(TURN_SPEED, EAST);
                break;
            case 3:
                turnRobot(EAST,1);
                driveRobot(20,EAST);
                break;
        }

        //_liftclaw.Calibrate();

        //turnToHeading(TURN_SPEED, 90); // turn ccw 90 degrees

        //driveLateral(DRIVE_SPEED, 24, RIGHT); //

        //turnToHeading( TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
        //holdHeading( TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second


        //driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
        //turnToHeading( TURN_SPEED,  45.0);               // Turn  CCW  to  45 Degrees
        //holdHeading( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

        //driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
        //turnToHeading( TURN_SPEED,   0.0);               // Turn  CW  to 0 Degrees
        //holdHeading( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second

        //driveStraight(DRIVE_SPEED,-48.0, 0.0);    // Drive in Reverse 48" (should return to approx. staring position)

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void driveRobot(double distance, double heading) {
        driveStraight(DRIVE_SPEED,distance,heading);
    }
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftgetCurrentPosition() + moveCounts;
            rightTarget = rightgetCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftsetTargetPosition(leftTarget);
            rightsetTargetPosition(rightTarget);

            leftsetMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightsetMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (leftisBusy() && rightisBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftsetMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightsetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /*
    This function is meant to do both turnToHeading and holdHeading.
     */
    public void turnRobot(double direction, double holdtime) {
        turnToHeading(TURN_SPEED, direction);
        holdHeading(TURN_SPEED,direction,holdtime);
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
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

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftsetPower(leftSpeed);
        rightsetPower(rightSpeed);
    }


    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftgetCurrentPosition(),
                    rightgetCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void leftsetMode(DcMotor.RunMode run) {
        motors[3].setMode(run);
        motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void rightsetMode(DcMotor.RunMode run) {
        motors[0].setMode(run);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void leftsetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motors[2].setZeroPowerBehavior(behavior);
        motors[3].setZeroPowerBehavior(behavior);
    }

    public void rightsetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motors[0].setZeroPowerBehavior(behavior);
        motors[1].setZeroPowerBehavior(behavior);
    }

    public int leftgetCurrentPosition() {
        return motors[2].getCurrentPosition();
    }
    public int rightgetCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    public void leftsetTargetPosition(int leftTarget) {
        //motors[3].setTargetPosition(leftTarget);
        motors[2].setTargetPosition(leftTarget);
    }

    public void rightsetTargetPosition(int rightTarget) {
        //motors[0].setTargetPosition(rightTarget);
        motors[0].setTargetPosition(rightTarget);
    }

    public boolean leftisBusy() {
        return motors[2].isBusy();
    }

    public boolean rightisBusy() {
        return motors[0].isBusy();
    }

    public void leftsetPower(double leftspeed) {
        motors[3].setPower(leftspeed);
        motors[2].setPower(leftspeed);
    }

    public void rightsetPower(double rightspeed) {
        rightspeed *= 0.8;
        motors[0].setPower(rightspeed);
        motors[1].setPower(rightspeed);
    }

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };
    private static final String VUFORIA_KEY = VuforiaKey.Key;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    private void TensorFlowInit(int tfodmonitorid, Telemetry telemetry, WebcamName camera) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(camera);
        initTfod(tfodmonitorid);

        telemetry.addData("Count", 0);
        telemetry.addData("Image", 0);
        telemetry.addData("- Position (Row/Col)", 0);
        telemetry.addData("- Size (Width/Height)", 0);

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0/9.0);
        }
    }

    private void initVuforia(WebcamName cameraName) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.cameraName = cameraName;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(int monitorId) {
        TFObjectDetector.Parameters tfodParameters;
        if (monitorId < 0) {
            tfodParameters = new TFObjectDetector.Parameters();
        }
        else {
            tfodParameters = new TFObjectDetector.Parameters(monitorId);
        }
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
