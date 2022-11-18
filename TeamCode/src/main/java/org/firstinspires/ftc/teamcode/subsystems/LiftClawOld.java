package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftClawOld {

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


    public final static int BOTTOM_POS = 0;   //1120 ticks per revolution
    public final static int LOW_POS = 2300;
    public final static int MEDIUM_POS = 3600;
    public final static int HIGH_POS = 5300;


    Telemetry.Item _T_pos,_B_stop,_C_STAT,_T_release;

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
        _T_pos = telemetry.addData("Lift", 0);
        _B_stop = telemetry.addData("BSTOP",bottomstopSensor.isPressed());
        _T_release = telemetry.addData("release",false);
        //Calibrate();
    }

    public void Calibrate() {
        int delta = 5;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveM.setPower(-0.5);
        while (!bottomstopSensor.isPressed()) {
            DriveM.getCurrentPosition();
        }
        DriveM.setPower(0);
        reset_zero();
    }

    public void reset_zero() {
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _T_pos.setValue(getEncoder());
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

        double current = DriveM.getCurrentPosition();

        double max_power = 1.0;
        double kP = 1/5300;

        long currentTime = System.currentTimeMillis();
        long lastTime,timeStep;
        double integral=0.0;
        double error=0;
        double derivative=0;
        double previous_error=0;

        while ( Math.abs(target-current) > 20) {
            //get the time step
            lastTime = currentTime;
            currentTime = System.currentTimeMillis();
            timeStep = currentTime-lastTime;

            // get current value and calculate the error, integral error and derivative error
            current = DriveM.getCurrentPosition();
            error = (target-current);
            integral +=error*timeStep;

            derivative = (error-previous_error)/timeStep;


            double output = kP * error;

            //clamp output range to be { -1 -- -0.1 } and {0.1 -- 1} if abs(power)>0.05
            if (Math.abs(output) <= 0.05 ) {
                output = 0;
            }
            else
                if ( Math.abs(output) <0.1 ) {
                    output = Math.signum(output)*0.1;
                }
            DriveM.setPower(Range.clip(output,-1,1));
        }

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

    private boolean _optical_open=false;
    public boolean checkOptical() {
        if (!_optical_open) {
            if (releaseSensor.getDistance(DistanceUnit.MM) < 15) {
                _T_release.setValue("true");
                ClawOpen();
                _optical_open = true;
                return true;
            }
            _T_release.setValue("false");
            _optical_open = false;
        }
        return false;
    }

    public void ClawOpen() {
        clawServos[CLAW_A].setPosition(servopos_open);
        clawServos[CLAW_B].setPosition(1 - servopos_open);
        _C_STAT.setValue("open");
    }

    public void ClawCloseThreaded() {
        if (! _optical_open) {
            clawServos[CLAW_A].setPosition(1 - servopos_close);
            clawServos[CLAW_B].setPosition(servopos_close);
            _C_STAT.setValue("closed");
        }
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
