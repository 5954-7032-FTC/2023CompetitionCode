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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.LiftClawLinear;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.hardware.TensorFlow;
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

@Autonomous(name = "LeftAutoTest", group = "Test")
public class AutoDriveByGyroLinerLeftTest extends LinearOpMode {

    /* Declare OpMode members. */


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()

    static final int LEFT_ENCODER = 3;
    static final int RIGHT_ENCODER = 0;

    LiftClawLinear _liftclaw;
    Servo _arm_release;

    MecanumDriveByGyro _move;

    DcMotor [] motors;

    TensorFlow tensorFlow;
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

        MecanumDrive.Parameters driveParameters = new MecanumDrive.Parameters();
        driveParameters.motors = motors;
        driveParameters._ENCODER_WHEELS=new int[]{0,3};
        driveParameters._REVERSED_WHEELS=new int[]{2,3};
        driveParameters.robotCentric=true;
        driveParameters.imu = hardwareMap.get(BNO055IMU.class, "imu");
        _move = new MecanumDriveByGyro(driveParameters,LEFT_ENCODER,RIGHT_ENCODER);


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

        AutoTransitioner.transitionOnStop(this, "TeleOp");
        double maxConfidence=-1;
        String maxLabel = null;

        // Wait for the game to start (Display Gyro value while waiting)
        while (!opModeIsActive()) {
            idle();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        _move.resetHeading();

        waitForStart();

        //is there a location?
        ElapsedTime et = new ElapsedTime();
        //we have 12 secs to look for the cone

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
        if ( maxConfidence <0  || maxLabel == null) {
            _whereToEnd_value = (int)Math.floor(Math.random()*100%3)+1; // 10 points on park
        }
        else {
            _whereToEnd_value = maxLabel.getBytes()[0] - '0'; // 3.3 points on average.
        }
        telemetry.addData("Endpoint:",_whereToEnd_value);


        final double NORTH=0;
        final double EAST=-90;
        final double WEST=90;
        final double SOUTH=180;

        _move.slideXY(0,18); // go forward 18"
        //_move.slideXY(18,18);

        //_move.driveRobot(19.5, NORTH);    // Drive Forward 18"
        //holdHeading(TURN_SPEED, NORTH, 1);
        _arm_release.setPosition(0.85); // release the arm.
        switch (_whereToEnd_value) {
            case 1:
                //_move.driveRobot( -16, EAST);
                _move.slideXY(-18,18);
                _move.turnRobot(EAST,1);
                break;
            case 2:
                _move.turnRobot(EAST, 1);
                break;
            case 3:
                _move.slideXY(18,18);
                _move.turnRobot(EAST,1);
                break;
        }

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
