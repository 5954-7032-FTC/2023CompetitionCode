package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ArmRelease;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.LiftClaw;
import org.firstinspires.ftc.teamcode.hardware.Lights;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotDevices;
import org.firstinspires.ftc.teamcode.util.GamepadEmpty;

public abstract class AutoLinearBase extends LinearOpMode {

    protected static final int HOLD_TIME = 1;
    protected int current_stack_height=LiftClaw.STACK_TOP_PICKUP;

    protected RobotDevices robotDevices;
    protected LiftClaw _liftclaw;
    protected ArmRelease armRelease;

    protected MecanumDrive _move;


    protected Lights light;

    protected final double LEFT=-90;
    protected final double RIGHT=90;
    protected static final int LEFT_SIDE=1;
    protected static final int RIGHT_SIDE=2;

    protected ColorSensorDevice colorSensorDeviceLeft, colorSensorDeviceRight;


    public abstract ColorSensorDevice getColorSensorDevice();
    public abstract void strafeDirection(double distance);
    public abstract void strafeAntiDirection(double distance);
    public abstract double turnDirection();
    public abstract void lightOn();
    public abstract void transitionOnStop();
    public abstract int side();

    public abstract Lights getLight();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        robotDevices = RobotDevices.getDevices(hardwareMap);


        light = getLight();// new Lights(hardwareMap.dcMotor.get("LIGHTS"));

        // set up MovementThread

        colorSensorDeviceLeft = new ColorSensorDevice(robotDevices.colorSensorLeft);
        colorSensorDeviceRight = new ColorSensorDevice(robotDevices.colorSensorRight);

        MecanumDrive.Parameters driveParameters = new MecanumDrive.Parameters();
        driveParameters.motors = robotDevices.wheels;
        driveParameters._ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters._REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.robotCentric = true;
        driveParameters.telemetry = telemetry;
        _move = new MecanumDrive(driveParameters);

        // setup LiftClaw
        LiftClaw.LiftClawParameters params = new LiftClaw.LiftClawParameters();
        params.DriveMotor = robotDevices.lift_motor;
        params.clawServos = robotDevices.lift_servos;
        params.gamepad = new GamepadEmpty();
        params.light = light;
        params.telemetry = telemetry;
        params.bottomStopSensor = robotDevices.bottom_stop;
        params.releaseSensor = robotDevices.post_sensor;

        _liftclaw = new LiftClaw(params);

        armRelease = robotDevices.arm_release;

        //transitionOnStop();

        waitForStart();

        lightOn();

        // start moving the bot.

        switch (side()) {
            case LEFT_SIDE:
                runLeft();
                break;
            case RIGHT_SIDE:
                runRight();
                break;
        }
    }

    public void runLeft() throws InterruptedException {
        // first put the arm up.
        armRelease.release();
        _liftclaw.calibrateLift();
        Thread.sleep(1500);
        _liftclaw.runToPos(LiftClaw.LOW_POS);

        double forward_amount=0,
                strafe_amount=0;

        // move to the cone
        strafeDirection(22); // should put us clearly on the cone
        //get the place to end from the findMaxColor()
        int place_to_end = getColorSensorDevice().findMaxColor();
        telemetry.log().add("Place to end: "+place_to_end);
        telemetry.update();
        // now to move around a bit...
        // place the first cone
        strafeDirection(22.5);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        // now get a new one.....
        strafeDirection(10.5);
        //turnRobot(0);
        driveForward(24.5);

        pickNextCone();
        // place it on the pole
        driveReverse(25);
        strafeAntiDirection(10.5);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        switch (place_to_end) {
            case 1:
                strafeDirection(14);
                driveReverse(24);
                break;
            case 3:
                strafeDirection(14);
                driveForward(24);
                break;
        }
        turnRobot(turnDirection());
        lightOff();
        requestOpModeStop();
    }

    public void runRight() throws InterruptedException {
        // first put the arm up.
        armRelease.release();
        _liftclaw.calibrateLift();
        Thread.sleep(1500);
        _liftclaw.runToPos(LiftClaw.LOW_POS);

        double forward_amount=0,
                strafe_amount=0;

        // move to the cone
        strafeDirection(22); // should put us clearly on the cone
        //get the place to end from the findMaxColor()
        int place_to_end = getColorSensorDevice().findMaxColor();
        telemetry.log().add("Place to end: "+place_to_end);
        telemetry.update();
        // now to move around a bit...
        // place the first cone
        strafeDirection(23.5);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        // now get a new one.....
        strafeDirection(13.5);
        //turnRobot(0);
        driveForward(26);

        pickNextCone();
        // place it on the pole
        driveReverse(26);
        strafeAntiDirection(13.5);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        switch (place_to_end) {
            case 1:
                strafeDirection(14);
                driveReverse(24);
                break;
            case 3:
                strafeDirection(14);
                driveForward(24);
                break;
        }
        turnRobot(turnDirection());
        lightOff();
        requestOpModeStop();
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
    }

    public void pickNextCone() {
        _liftclaw.clawClose();
        _liftclaw.runToPos(current_stack_height); // pick up cone
        _liftclaw.runToPos(LiftClaw.LOW_POS);
        updateStackHeight();
    }

    public void placeCone(long pos) {
        _liftclaw.clawOpen();
        /*
        _liftclaw.placeCone();
        _liftclaw.clawOpen();
        _liftclaw.runToPos(LiftClaw.LOW_POS);
         */
    }

    public void turnRobot(double direction) {
        //_move.turnRobot(direction,HOLD_TIME);
    }

    public void updateStackHeight() {
        current_stack_height -= LiftClaw.STACK_INCREMENT;
    }

}