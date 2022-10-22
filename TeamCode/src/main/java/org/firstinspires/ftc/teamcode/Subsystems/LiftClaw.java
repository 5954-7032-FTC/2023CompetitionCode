package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftClaw {

    DcMotor DriveM;
    Servo[] clawServos;
    TouchSensor bottomstopSensor;
    DistanceSensor releaseSensor;
    Telemetry telemetry;

    final static int CLAW_OPEN = 0;
    final static int CLAW_CLOSE = 1;
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;

    final static int CALIBRATE = 0;
    public final static int BOTTOM = 1;
    public final static int LOW = 2;
    public final static int MEDIUM = 3;
    public final static int HIGH = 4;


    final static int BOTTOM_POS = 0;
    final static int LOW_POS = 100;
    final static int MEDIUM_POS = 200;
    final static int HIGH_POS = 300;


    /******************************************
     * Initialize with a motor array
     * @param Motor Define motors
     */
    public void Initialize(DcMotor Motor, Servo servos[], TouchSensor stop, DistanceSensor release, Telemetry telemetry) {
        this.telemetry = telemetry;
        DriveM = Motor;
        //DriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServos = new Servo[2];
        for (int i = 0; i < servos.length; i++) {
            servos[i].setDirection(Servo.Direction.FORWARD);
            clawServos[i] = servos[i];
        }
        ClawOpen();
        ClawClose();
        bottomstopSensor = stop;
        releaseSensor = release;

        Calibrate();
    }

    public void Calibrate() {
        int last = -1;
        int current = 0;

        int delta = 5;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        current = DriveM.getCurrentPosition();
        DriveM.setPower(1);
        DriveM.setPower(-0.5);
        while (!bottomstopSensor.isPressed()) {
            last = current;
            current = DriveM.getCurrentPosition();
        }
        DriveM.setPower(0);
        reset_zero();
    }

    public void reset_zero() {
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void testBottom() {
        telemetry.addData("Bottom sensor", bottomstopSensor.isPressed());
    }


    public double Move(int location) {
        switch (location) {
            case BOTTOM:
                DriveM.setTargetPosition(BOTTOM_POS);
                break;
            case LOW:
                DriveM.setTargetPosition(LOW_POS);
                break;
            case MEDIUM:
                DriveM.setTargetPosition(MEDIUM_POS);
                break;
            case HIGH:
                DriveM.setTargetPosition(HIGH_POS);
                break;
        }
        return 0;
    }

    public double Move(double move) {
        if (!bottomstopSensor.isPressed()) {
            DriveM.setPower(Range.clip(-move, -1, 1));
        } else {
            reset_zero();
            if (-move > 0) {
                DriveM.setPower(Range.clip(-move, -1, 1));
            } else {
                DriveM.setPower(0);
            }
        }
        telemetry.addData("Position", (getEncoder()));
        telemetry.update();

        return move;
    }

    double servopos_open = 0;
    double servopos_close = .45;

    double opentime = 0.0;

    public void checkOptical() {
        if (releaseSensor.getDistance(DistanceUnit.MM) < 15) {
            ClawOpen();
            opentime = System.currentTimeMillis();
        }
    }

    public void ClawOpen() {
        clawServos[CLAW_A].setPosition(servopos_open);
        clawServos[CLAW_B].setPosition(1 - servopos_open);
        telemetry.addData("CLAW_A_pos", clawServos[CLAW_A].getPosition());
        telemetry.addData("CLAW_B_pos", clawServos[CLAW_B].getPosition());
        telemetry.update();
    }

    public void ClawClose() {
        if (opentime != 0) {
            if (System.currentTimeMillis() - opentime <= 1000) {
                return;
            }
            opentime = 0;
        }
        clawServos[CLAW_A].setPosition(1 - servopos_close);
        clawServos[CLAW_B].setPosition(servopos_close);
        telemetry.addData("CLAW_A_pos", clawServos[CLAW_A].getPosition());
        telemetry.addData("CLAW_B_pos", clawServos[CLAW_B].getPosition());
        telemetry.update();
    }

    private double deadzone(double power) {
        return Math.abs(power) > 0.1 ? power : 0.0;
    }

    public double getEncoder() {
        return DriveM.getCurrentPosition();
    }

    public final static double ENC_SCALE = Math.PI * 7 / (8 * 280);

    public double getPosition() {
        return getEncoder() * ENC_SCALE;
    }

    public boolean HasEncodersReset() {
        return getEncoder() == 0;
    }

    public void SetMode(DcMotor.RunMode SMode) {
        DriveM.setMode(SMode);
    }

}
