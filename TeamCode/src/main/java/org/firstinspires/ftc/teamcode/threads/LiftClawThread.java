package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftClawThread extends RobotThread {
    private final Gamepad _gamepad;
    //constants
    final static double _servo_pos_open = 0;
    final static double _servo_pos_close = .45;
    final static double _servo_pos_nut = .72;
    public final static int BOTTOM_POS = 0;   //1120 ticks per revolution
    public final static int LOW_POS = 2300;
    public final static int MEDIUM_POS = 3600;
    public final static int HIGH_POS = 5475;
    // static points
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;


    // hardware objects
    DcMotor _DriveMotor;
    Servo[] _clawServos;
    TouchSensor _bottomStopSensor;
    DistanceSensor _releaseSensor;
    Telemetry _telemetry;

    //telemetry items
    Telemetry.Item _T_pos,_B_stop,_C_STAT;



    //34.75" from bottom to top
    /*
    1540 top of 5 stack


   cone stack heights
     1st 0
     2nd 200
     3rd 400
     4th 600
     5th 800
     */

    public LiftClawThread(DcMotor Motor, Servo [] servos, TouchSensor stop, DistanceSensor release,
                          Telemetry telemetry, Gamepad gamepad) {
        _gamepad=gamepad;
        _telemetry = telemetry;
        _DriveMotor = Motor;
        _clawServos = new Servo[2];
        for (int i = 0; i < servos.length; i++) {
            servos[i].setDirection(Servo.Direction.FORWARD);
            _clawServos[i] = servos[i];
        }

        _C_STAT = telemetry.addData("Claw", "open");
        ClawOpen();
        ClawClose();
        _bottomStopSensor = stop;
        _releaseSensor = release;
        _T_pos = telemetry.addData("Lift", 0);
        _B_stop = telemetry.addData("BSTOP", _bottomStopSensor.isPressed());
        _DriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Calibrate();
    }


    public void ClawOpen() {
        _clawServos[CLAW_A].setPosition(_servo_pos_open);
        _clawServos[CLAW_B].setPosition(1-_servo_pos_open);
        _C_STAT.setValue("open");
    }

    long _clawOpenTime=0;
    public void ClawClose() {
        if (System.currentTimeMillis() > _clawOpenTime+1500) {
            _clawServos[CLAW_A].setPosition(1 - _servo_pos_close);
            _clawServos[CLAW_B].setPosition(_servo_pos_close);
            _C_STAT.setValue("closed");
        }
    }

    public void ClawCloseNut() {
            _clawServos[CLAW_A].setPosition(1 - _servo_pos_nut);
            _clawServos[CLAW_B].setPosition(_servo_pos_nut);
            _C_STAT.setValue("closedNUT");
    }

    public int getEncoder() {
        return _DriveMotor.getCurrentPosition();
    }

    public void SetMode(DcMotor.RunMode SMode) {
        _DriveMotor.setMode(SMode);
    }

    public void Move(double move) { // move with the joystick
        if (!_bottomStopSensor.isPressed()) {
            _DriveMotor.setPower(Range.clip(-move, -1, 1));
        } else {
            reset_zero();
            if (-move > 0) {
                _DriveMotor.setPower(Range.clip(-move, -1, 1));
            } else {
                _DriveMotor.setPower(0);
            }
        }
        _T_pos.setValue(getEncoder());
    }

    public void Calibrate() {

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!_bottomStopSensor.isPressed()) {
            _DriveMotor.setPower(-0.5);
            //_DriveMotor.getCurrentPosition();
        }
        _DriveMotor.setPower(0);
        reset_zero();
    }

    public void reset_zero() {
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _T_pos.setValue(getEncoder());
    }


    public boolean checkOptical() {
        return (_releaseSensor.getDistance(DistanceUnit.MM) < 30);
    }

    public void runToPos(int target) {

        int current, last;
       SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        _DriveMotor.setTargetPosition(target);
        _DriveMotor.setPower( -Math.signum(getEncoder()-target) );
        last = -1;
        while (true) {
            current = _DriveMotor.getCurrentPosition();
            if (current == last) break;
            last = current;
            if (!_DriveMotor.isBusy()) break;
            if (Math.abs(_gamepad.left_stick_y) > 0.1) break;
        }
        _DriveMotor.setPower(0);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void placeCone() {
        _DriveMotor.setPower(-1);
        while (true) {
            if (checkOptical()) break;
            if (_DriveMotor.getCurrentPosition() < 100  || _DriveMotor.getCurrentPosition() > 5300) break;

        }
        _DriveMotor.setPower(0);
    }
    public boolean _is_auto = false;
    public void run() {
        Calibrate();
        while (!isCancelled()) {
            //lift movement
            if (checkOptical()) {
                _clawOpenTime = System.currentTimeMillis();
                ClawOpen();
            }
            if (_is_auto) continue;

            Move((double) _gamepad.left_stick_y);
            //  claw control
            if (_gamepad.left_trigger >0) {
                ClawCloseNut();
            }
            else {
                if (_gamepad.right_trigger > 0) {
                    ClawOpen();
                } else {
                    ClawClose();
                }
            }




            if (_gamepad.x) {
                runToPos(BOTTOM_POS);
            }
            if (_gamepad.a) {
                runToPos(LOW_POS);
            }
            if (_gamepad.b) {
                runToPos(MEDIUM_POS);
            }
            if (_gamepad.y) {
                runToPos(HIGH_POS);
            }
        }
    }
}
