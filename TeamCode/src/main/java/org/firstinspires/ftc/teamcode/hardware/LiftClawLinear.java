package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Deprecated
public class LiftClawLinear {


    public static class Positions {
        public final static int BOTTOM   = 0;
        public static final int LOW      = 2300;
        public static final int MEDIUM      = 3600;
        public static final int HIGH      = 5475;
        public static final int [] STACK_GRAB_HEIGHTS = {
                800,
                600,
                400,
                200,
                0
        };
    }

    //constants
    final static double _servo_pos_open = 0;
    final static double _servo_pos_close = .45;
    // static points
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;


    //34.75" from bottom to top
    /*
    1540 top of 5 stack


   cone stack grab heights
     1st 0
     2nd 200
     3rd 400
     4th 600
     5th 800
     */




    // hardware objects
    DcMotor[] _LiftMotors;
    Servo[] _clawServos;
    TouchSensor _bottomStopSensor;
    DistanceSensor _releaseSensor;
    Telemetry _telemetry;
    //telemetry items
    Telemetry.Item _T_pos;




    public LiftClawLinear(DcMotor [] liftMotors, Servo [] clawServos, TouchSensor stop, DistanceSensor release,
                          Telemetry telemetry) {
        _telemetry = telemetry;

        _LiftMotors = liftMotors;
        _LiftMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _LiftMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _bottomStopSensor = stop;
        _releaseSensor = release;

        _clawServos = clawServos;
        for (Servo servo: _clawServos)
            servo.setDirection(Servo.Direction.FORWARD);

        _T_pos = telemetry.addData("Lift:", 0);
    }


    public void ClawOpen() {
        _clawServos[CLAW_A].setPosition(_servo_pos_open);
        _clawServos[CLAW_B].setPosition(1 - _servo_pos_open);
    }

    long _clawOpenTime=0;
    public void ClawClose() {
        if (System.currentTimeMillis() > _clawOpenTime+1500) {
            _clawServos[CLAW_A].setPosition(1 - _servo_pos_close);
            _clawServos[CLAW_B].setPosition(_servo_pos_close);
        }
    }

    public int getEncoder() {
        return _LiftMotors[0].getCurrentPosition();
    }

    public void SetMode(DcMotor.RunMode SMode) {
        _LiftMotors[0].setMode(SMode);
    }


    public void Calibrate() {

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!_bottomStopSensor.isPressed()) {
            _LiftMotors[0].setPower(-0.5);
            _LiftMotors[1].setPower(-0.5);
            //_DriveMotor.getCurrentPosition();
        }
        _LiftMotors[0].setPower(0);
        _LiftMotors[1].setPower(0);
        reset_zero();
    }

    public void reset_zero() {
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _T_pos.setValue(getEncoder());
    }


    public boolean checkOptical() {
        return (_releaseSensor.getDistance(DistanceUnit.MM) < 15);
    }

    private boolean stop_runToPos = false;

    public synchronized void setStop_runToPos(boolean stop_runToPos) {
        this.stop_runToPos = stop_runToPos;
    }

    public void runToPos(int target) {
        int current, last;
       SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        _LiftMotors[0].setTargetPosition(target);
        _LiftMotors[0].setPower( -Math.signum(getEncoder()-target) );
        _LiftMotors[1].setPower( -Math.signum(getEncoder()-target) );
        last = -1;
        while (!stop_runToPos) {
            current = _LiftMotors[0].getCurrentPosition();
            if (current == last) break;
            last = current;
            if (!_LiftMotors[0].isBusy()) break;
            //if (Math.abs(_gamepad.left_stick_y) > 0.1) break;
        }
        _LiftMotors[0].setPower(0);
        _LiftMotors[1].setPower(0);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setStop_runToPos(false);
    }

    public void placeCone() {
        while (true) {
            if (checkOptical()) {
                _clawOpenTime = System.currentTimeMillis();
                ClawOpen();
            }

        }
    }

    public void Move(double move) { // move with the joystick
        if (!_bottomStopSensor.isPressed()) {
            _LiftMotors[0].setPower(Range.clip(-move, -1, 1));
            _LiftMotors[1].setPower(Range.clip(-move, -1, 1));
        } else { // bottom sensor is pressed
            reset_zero();
            if (-move > 0) {//if moving up, let it!
                _LiftMotors[0].setPower(Range.clip(-move, -1, 1));
                _LiftMotors[1].setPower(Range.clip(-move, -1, 1));
            } else { // else stop
                _LiftMotors[0].setPower(0);
                _LiftMotors[1].setPower(0);
            }
        }
        _T_pos.setValue(getEncoder());
    }

}
