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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ArmRelease;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.LiftClaw;
import org.firstinspires.ftc.teamcode.hardware.Lights;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveByGyro;

public abstract class GyroLinearBase extends LinearOpMode {

    protected static final int HOLD_TIME = 1;
    protected int current_stack_height=LiftClaw.STACK_TOP;
    protected LiftClaw _liftclaw;
    protected ArmRelease armRelease;

    protected MecanumDriveByGyro _move;

    protected DcMotor [] motors;

    protected Lights light;

    final double LEFT=-90;
    final double RIGHT=90;

    protected ColorSensorDevice colorSensorDevice;
    protected ColorSensorDevice colorSensorDeviceLeft, colorSensorDeviceRight;


    public abstract ColorSensorDevice getColorSensorDevice();
    public abstract void strafeDirection(double distance);
    public abstract void strafeAntiDirection(double distance);
    public abstract void lightOn();
    public abstract void transitionOnStop();

    public void initGyroLinearBase() {
        telemetry.setAutoClear(false);

        light = new Lights(hardwareMap.dcMotor.get("LIGHTS"));

        // set up MovementThread

        colorSensorDeviceRight = new ColorSensorDevice(hardwareMap.colorSensor.get("RIGHT_COLOR"));
        colorSensorDeviceLeft = new ColorSensorDevice(hardwareMap.colorSensor.get("LEFT_COLOR"));

        motors = new DcMotor[]{
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")
        };
        MecanumDrive.Parameters driveParameters = new MecanumDrive.Parameters();
        driveParameters.motors = motors;
        driveParameters._ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters._REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.robotCentric = true;
        driveParameters.imu = hardwareMap.get(BNO055IMU.class, "imu");
        driveParameters.telemetry = telemetry;
        _move = new MecanumDriveByGyro(driveParameters);

        // setup LiftClaw
        final DcMotor[] lift_motors = {
                hardwareMap.dcMotor.get("LIFT"),
                hardwareMap.dcMotor.get("LIFT2")
        };
        final Servo[] lift_servos = {
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };
        final TouchSensor bottom_stop = hardwareMap.touchSensor.get("BSTOP");
        final DistanceSensor post_sensor = hardwareMap.get(DistanceSensor.class, "C_STOP");


        final Servo pipe_guide = hardwareMap.servo.get("PIPE_GUIDE");

        _liftclaw = new LiftClaw(
                lift_motors,
                lift_servos,
                pipe_guide,
                bottom_stop,
                post_sensor,
                telemetry,
                new Gamepad()
        );

        armRelease = new ArmRelease(hardwareMap.servo.get("ARM_RELEASE"));

        transitionOnStop();

        double maxConfidence = -1;
        String maxLabel = null;
        _move.resetHeading();

    }

    //TensorFlow tensorFlow;
    @Override
    public void runOpMode() throws InterruptedException {

        initGyroLinearBase();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        lightOn();

        // start moving the bot.

        // first put the arm up.
        armRelease.release();
        // move to the cone
        strafeDirection(20); // should put us clearly on the cone
        //get the place to end from the findMaxColor()
        int place_to_end = getColorSensorDevice().findMaxColor();
        // now to move around a bit...
        // place the first cone
        _liftclaw.runToPos(LiftClaw.LOW_POS);
        strafeDirection(12);
        driveForward(1);
        placeCone();
        // now get a new one.....
        strafeDirection(12);
        driveForward(24);
        pickNextCone();
        // place it on the pole
        driveReverse(24);
        strafeAntiDirection(12);
        placeCone();

        switch (place_to_end) {
            case 1:
                strafeDirection(12);
                driveForward(24);
                break;
            case 2:
                // already there!
                break;
            case 3:
                strafeDirection(12);
                driveReverse(24);
                break;
        }
        turnRobot(LEFT);
        lightOff();
    }

    public void driveForward(double distanceInches) {
        _move.driveForward(distanceInches);
    }

    public void driveLeft(double distanceInches) {
        _move.driveLeft(distanceInches);
    }

    public void driveReverse(double distanceInches) {
        _move.driveReverse(distanceInches);
    }

    public void driveRight(double distanceInches) {
        _move.driveRight(distanceInches);
    }

    public void lightOff() {
        light.off();
    };

    public void pickNextCone() {
        _liftclaw.clawClose();
        _liftclaw.runToPos(current_stack_height); // pick up cone
        _liftclaw.runToPos(LiftClaw.LOW_POS);
        updateStackHeight();
    }

    public void placeCone() {
        _liftclaw.placeCone();
        _liftclaw.runToPos(40);    // place cone
        _liftclaw.clawOpen();
        _liftclaw.runToPos(LiftClaw.LOW_POS);
    }

    public void turnRobot(double direction) {
        _move.turnRobot(direction,HOLD_TIME);
    }

    public void updateStackHeight() {
        current_stack_height -= LiftClaw.STACK_INCREMENT;
    }

}