package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.teamcode.hardware.LiftClaw;

public class LiftClawThread extends RobotThread {
    private final Gamepad _gamepad;
    //constants
    /*
    final static double _servo_pos_open = 0;
    final static double _servo_pos_close = .45;
    final static double _servo_pos_nut = .72;
    public final static int BOTTOM_POS = 0;   //1120 ticks per revolution
    public final static int LOW_POS = 2300;
    public final static int MEDIUM_POS = 3600;
    public final static int HIGH_POS = 5475;

    public final static double PIPE_GUIDE_OPEN = 0.95;
    public final static double PIPE_GUIDE_CLOSE =0.0;
    final static long CLAW_GUIDE_WAIT=2000;

    public final static int GUIDE_UP_HEIGHT=2400;
    public final static int GUIDE_DOWN_HEIGHT=3500;

    // static points
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;
*/

    // hardware objects
    //DcMotor[]  _DriveMotors;
    //Servo[] _clawServos;
    //Servo _pipe_guide;
    //TouchSensor _bottomStopSensor;
    //DistanceSensor _releaseSensor;
//    Telemetry _telemetry;
    //telemetry items
//    Telemetry.Item _T_pos,_B_stop,_C_STAT,P_GUIDE;
    LiftClaw _claw;


    public LiftClawThread(DcMotor [] Motors, Servo [] servos, Servo pipe_guide, TouchSensor stop, DistanceSensor release,
                          Telemetry telemetry, Gamepad gamepad) {
        _gamepad=gamepad;
  //      _telemetry = telemetry;
        //_DriveMotors = Motors;
        //_clawServos = new Servo[2];
        //_pipe_guide = pipe_guide;
        //for (int i = 0; i < servos.length; i++) {
        //    servos[i].setDirection(Servo.Direction.FORWARD);
        //    _clawServos[i] = servos[i];
       // }

        //_C_STAT = telemetry.addData("Claw", "open");
        //ClawOpen();
        //ClawClose();
        //_bottomStopSensor = stop;
        //_releaseSensor = release;
        //_DriveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //_DriveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //_DriveMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        _claw = new LiftClaw(Motors,servos,pipe_guide,stop,release,telemetry,gamepad);

        //Calibrate();
    }

/*
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

    public void pipeGuideUp() {
        _pipe_guide.setPosition(PIPE_GUIDE_OPEN);
        P_GUIDE.setValue("UP");
    }

    public void pipeGuideDown() {
        if (System.currentTimeMillis() > _clawOpenTime+CLAW_GUIDE_WAIT+0.3) {
            _pipe_guide.setPosition(PIPE_GUIDE_CLOSE);
            P_GUIDE.setValue("DOWN");
        }
    }

    public void ClawCloseNut() {
            _clawServos[CLAW_A].setPosition(1 - _servo_pos_nut);
            _clawServos[CLAW_B].setPosition(_servo_pos_nut);
            _C_STAT.setValue("closedNUT");
    }

    public int getEncoder() {
        return _DriveMotors[0].getCurrentPosition();
    }

    public void SetMode(DcMotor.RunMode SMode) {
        _DriveMotors[0].setMode(SMode);
    }

    public void Move(double move) { // move with the joystick
        if (!_bottomStopSensor.isPressed()) {
            _DriveMotors[0].setPower(Range.clip(-move, -1, 1));
            _DriveMotors[1].setPower(Range.clip(-move, -1, 1));
        } else {
            reset_zero();
            if (-move > 0) {
                _DriveMotors[0].setPower(Range.clip(-move, -1, 1));
                _DriveMotors[1].setPower(Range.clip(-move, -1, 1));
            } else {
                _DriveMotors[0].setPower(0);
                _DriveMotors[1].setPower(0);
            }
        }
        _T_pos.setValue(getEncoder());
    }

    public void Calibrate() {

        pipeGuideUp();
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!_bottomStopSensor.isPressed()) {
            _DriveMotors[0].setPower(-0.5);
            _DriveMotors[1].setPower(-0.5);
            //_DriveMotor.getCurrentPosition();
        }
        _DriveMotors[0].setPower(0);
        _DriveMotors[1].setPower(0);
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
        _DriveMotors[0].setTargetPosition(target);
        _DriveMotors[0].setPower( -Math.signum(getEncoder()-target) );
        _DriveMotors[1].setPower( -Math.signum(getEncoder()-target) );
        last = -1;
        while (true) {
            current = _DriveMotors[0].getCurrentPosition();
            if (current == last) break;
            last = current;
            if (!_DriveMotors[0].isBusy()) break;
            if (Math.abs(_gamepad.left_stick_y) > 0.1) break;
        }
        _DriveMotors[0].setPower(0);
        _DriveMotors[1].setPower(0);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void placeCone() {
        _DriveMotors[0].setPower(-1);
        _DriveMotors[1].setPower(-1);
        while (true) {
            if (checkOptical()) break;
            if (_DriveMotors[0].getCurrentPosition() < 100  || _DriveMotors[0].getCurrentPosition() > 5300) break;

        }
        _DriveMotors[0].setPower(0);
        _DriveMotors[1].setPower(0);
    }
    public boolean _is_auto = false;
*/

    public void run() {
        _claw.calibrateLift();
        while (!isCancelled()) {
            //lift movement
            if (_claw.checkOptical()) {
                _claw.setClawOpenTime(System.currentTimeMillis());
                _claw.clawOpen();
            }

            _claw.moveLift((double) _gamepad.left_stick_y);
            //  claw control
            if (_gamepad.right_trigger > 0) {
                _claw.clawOpen();
            } else {
                _claw.clawClose();
            }
            if (_gamepad.x) {
                _claw.runToPos(LiftClaw.BOTTOM_POS);
            }
            if (_gamepad.a) {
                _claw.runToPos(LiftClaw.LOW_POS);
            }
            if (_gamepad.b) {
                _claw.runToPos(LiftClaw.MEDIUM_POS);
            }
            if (_gamepad.y) {
                _claw.runToPos(LiftClaw.HIGH_POS);
            }



            long pos = _claw.getEncoder();


            if (_gamepad.left_bumper) _claw.pipeGuideUp();
            else
            if (_gamepad.right_bumper) _claw.pipeGuideDown();
            else {
                if (pos < LiftClaw.GUIDE_UP_HEIGHT)
                    _claw.pipeGuideUp();
                if (pos > LiftClaw.GUIDE_DOWN_HEIGHT)
                    _claw.pipeGuideDown();
            }
        }
    }
}
