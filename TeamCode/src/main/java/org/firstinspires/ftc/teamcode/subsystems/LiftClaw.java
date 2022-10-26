package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    final static int CLAW_A = 0;
    final static int CLAW_B = 1;

    public final static int BOTTOM = 1;
    public final static int LOW = 2;
    public final static int MEDIUM = 3;
    public final static int HIGH = 4;


    public final static int BOTTOM_POS = 0;
    public final static int LOW_POS = 3561;
    public final static int MEDIUM_POS = 5879;
    public final static int HIGH_POS = 8015;


    Telemetry.Item _T_pos,_B_stop,_C_STAT;

    /******************************************
     * Initialize with a motor array
     * @param Motor Define motors
     */
    public void Initialize(DcMotor Motor, Servo [] servos, TouchSensor stop, DistanceSensor release, Telemetry telemetry) {
        this.telemetry = telemetry;
        DriveM = Motor;
        //DriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServos = new Servo[2];
        for (int i = 0; i < servos.length; i++) {
            servos[i].setDirection(Servo.Direction.FORWARD);
            clawServos[i] = servos[i];
        }

        _C_STAT = telemetry.addData("Claw", "open");
        ClawOpen();
        ClawClose();
        bottomstopSensor = stop;
        releaseSensor = release;

        Calibrate();
        _T_pos = telemetry.addData("Lift", 0);
        _B_stop = telemetry.addData("BSTOP",bottomstopSensor.isPressed());
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
        _T_pos.setValue(getEncoder());
    }


    public void testBottom() {
        telemetry.addData("Bottom sensor", bottomstopSensor.isPressed());
    }


    public double target = -1;

    public double Move(int location) {  // move to a set position
        switch (location) {
            case BOTTOM:
                target = BOTTOM_POS;
                break;
            case LOW:
                target = LOW_POS;
                break;
            case MEDIUM:
                target = MEDIUM_POS;
                break;
            case HIGH:
                target = HIGH_POS;
                break;
        }
        double diff;
        do {
            diff=DriveM.getCurrentPosition()-target;
            DriveM.setPower(Math.signum(diff));
            } while (diff < 30 );
        DriveM.setPower(0);
        target = -1;
        return 0;
    }

    public double Move(double move) { // move with the joystick
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
        _T_pos.setValue(getEncoder());
        return move;
    }

    double servopos_open = 0;
    double servopos_close = .45;

    double ClawOpenSince = 0.0;

    public boolean checkOptical() {
        if (releaseSensor.getDistance(DistanceUnit.MM) < 15) {
            ClawOpen();
            ClawOpenSince = System.currentTimeMillis();
            return true;
        }
        return false;
    }

    public void ClawOpen() {
        clawServos[CLAW_A].setPosition(servopos_open);
        clawServos[CLAW_B].setPosition(1 - servopos_open);
        _C_STAT.setValue("open");
    }

    public void ClawCloseThreaded() {
        clawServos[CLAW_A].setPosition(1 - servopos_close);
        clawServos[CLAW_B].setPosition(servopos_close);
        _C_STAT.setValue("closed");
    }

    public void ClawClose() {
        if ( (ClawOpenSince != 0)   && (System.currentTimeMillis() - ClawOpenSince <= 1000)  ){
                return;
            }
        ClawOpenSince = 0;
        ClawCloseThreaded();
    }


    public double getEncoder() {
        return DriveM.getCurrentPosition();
    }

    public void SetMode(DcMotor.RunMode SMode) {
        DriveM.setMode(SMode);
    }

}
