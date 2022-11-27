package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.threads.RobotThread;
import org.firstinspires.ftc.teamcode.util.GamepadEmpty;

public class LiftClaw extends RobotThread {
    private final Gamepad _gamepad;
    //constants
    final static double _servo_pos_open = 0;
    final static double _servo_pos_close = .45;
    public final static int BOTTOM_POS = 0;   //1120 ticks per revolution
    public final static int LOW_POS = 2300;
    public final static int MEDIUM_POS = 3600;
    public final static int HIGH_POS = 5475;

    public final static double PIPE_GUIDE_OPEN = 0.95;
    public final static double PIPE_GUIDE_CLOSE =0.0;
    final static long CLAW_GUIDE_WAIT=2000;

    public final static int GUIDE_UP_HEIGHT=2400;
    public final static int GUIDE_DOWN_HEIGHT=3500;

    public final static int STACK_TOP=1540;
    public final static int STACK_TOP_PICKUP=800;
    public final static int STACK_INCREMENT=200;

    // static points
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;


    // hardware objects
    DcMotor _DriveMotor;
    Servo[] _clawServos;
    Servo _pipe_guide;
    TouchSensor _bottomStopSensor;
    DistanceSensor _releaseSensor;
    Telemetry _telemetry;
    Lights _light;

    //telemetry items
    Telemetry.Item _T_pos,_B_stop,_C_STAT,P_GUIDE;
    long _clawOpenTime=0;


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
    public LiftClaw(DcMotor Motor, Servo [] servos, Servo pipe_guide, TouchSensor stop,
                    DistanceSensor release, Telemetry telemetry, Gamepad gamepad, Lights light) {
        _light = light;
        _gamepad=gamepad;
        _telemetry = telemetry;
        _DriveMotor = Motor;
        _clawServos = new Servo[2];
        _pipe_guide = pipe_guide;
        for (int i = 0; i < servos.length; i++) {
            servos[i].setDirection(Servo.Direction.FORWARD);
            _clawServos[i] = servos[i];
        }

        _C_STAT = telemetry.addData("Claw", "open");
        clawOpen();
        clawClose();
        _bottomStopSensor = stop;
        _releaseSensor = release;
        _T_pos = telemetry.addData("Lift", 0);
        _B_stop = telemetry.addData("BSTOP", _bottomStopSensor.isPressed());
        _DriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        P_GUIDE = _telemetry.addData("PIPE GUIDE", "unknown");
        //Calibrate();
    }

    public void calibrateLift() {

        pipeGuideUp();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!_bottomStopSensor.isPressed()) {
            _DriveMotor.setPower(-0.5);
            //_DriveMotor.getCurrentPosition();
        }
        _DriveMotor.setPower(0);
        resetEncoder();
    }

    public boolean checkOptical() {
        return (_releaseSensor.getDistance(DistanceUnit.MM) < 30);
    }

    public void checkPipe() {
        long current = getEncoder();
        if (current < GUIDE_UP_HEIGHT) pipeGuideUp();
        if (current > GUIDE_DOWN_HEIGHT) pipeGuideDown();
    }

    public void clawClose() {
        if (System.currentTimeMillis() > _clawOpenTime+1500) {
            _clawServos[CLAW_A].setPosition(1 - _servo_pos_close);
            _clawServos[CLAW_B].setPosition(_servo_pos_close);
            _C_STAT.setValue("closed");
        }
    }

    public void clawOpen() {
        _clawServos[CLAW_A].setPosition(_servo_pos_open);
        _clawServos[CLAW_B].setPosition(1-_servo_pos_open);
        _C_STAT.setValue("open");
    }

    public int getEncoder() {
        return _DriveMotor.getCurrentPosition();
    }

    public void moveLift(double speed) { // move with the joystick
        if (!_bottomStopSensor.isPressed()) {
            _DriveMotor.setPower(Range.clip(-speed, -1, 1));
        } else {
            resetEncoder();
            if (-speed > 0) {
                _DriveMotor.setPower(Range.clip(-speed, -1, 1));
            } else {
                _DriveMotor.setPower(0);
            }
        }
        checkPipe();
        _T_pos.setValue(getEncoder());
    }

    public void pipeGuideUp() {
        //_pipe_guide.setPosition(PIPE_GUIDE_OPEN);
        P_GUIDE.setValue("UP");
    }

    public void pipeGuideDown() {
        if (System.currentTimeMillis() > _clawOpenTime+CLAW_GUIDE_WAIT+0.3) {
            //_pipe_guide.setPosition(PIPE_GUIDE_CLOSE);
            P_GUIDE.setValue("DOWN");
        }
    }

    public void placeCone() {
        _DriveMotor.setPower(-1);
        while (true) {
            if (checkOptical()) break;
            if (_DriveMotor.getCurrentPosition() < 100  || _DriveMotor.getCurrentPosition() > 5300) break;

        }
        clawOpen();
        _DriveMotor.setPower(0);
    }

    public void resetEncoder() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _T_pos.setValue(getEncoder());
    }

    public void runToPos(int target) {
        int current, last;
        _DriveMotor.setTargetPosition(target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(_DriveMotor, -Math.signum(getEncoder()-target));
        last = -100;
        ElapsedTime et = new ElapsedTime();
        et.reset();
        long start_time = System.currentTimeMillis();
        while (true) {
            current = _DriveMotor.getCurrentPosition();
            checkPipe();

            if ((System.currentTimeMillis() > start_time + 100) && (current == last)) {
                _telemetry.log().add("current("+current+") == last("+last+")");
                break;
            }
            last = current;
            if (!_DriveMotor.isBusy()) {
                _telemetry.log().add("motor is not busy");
                break;
            }
            if (!( _gamepad instanceof GamepadEmpty ) && (Math.abs(_gamepad.left_stick_y) > 0.1)) {
                _telemetry.log().add("GAMEPAD TRIGGERED A STOP?");
                break;
            }
        }
        setPower(_DriveMotor,0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setClawOpenTime(long _clawOpenTime) {
        this._clawOpenTime = _clawOpenTime;
    }

    public void setMode(DcMotor.RunMode mode) {
        _DriveMotor.setMode(mode);
    }

    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }


}
