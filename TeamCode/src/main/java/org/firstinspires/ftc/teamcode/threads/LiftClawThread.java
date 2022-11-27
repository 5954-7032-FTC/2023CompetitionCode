package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.teamcode.hardware.LiftClaw;
import org.firstinspires.ftc.teamcode.hardware.Lights;

public class LiftClawThread extends RobotThread {
    private final Gamepad _gamepad;


    LiftClaw _claw;

    Lights _light;

    public LiftClawThread(DcMotor Motor, Servo [] servos, Servo pipe_guide, TouchSensor stop, DistanceSensor release,
                          Telemetry telemetry, Gamepad gamepad, Lights light) {
        _gamepad=gamepad;
        _claw = new LiftClaw(Motor,servos,pipe_guide,stop,release,telemetry,gamepad,light);

        _light = light;
        //Calibrate();
    }

    public void run() {
        _claw.calibrateLift();
        while (!isCancelled()) {
            //lift movement
            if (_claw.checkOptical()) {
                //_light.blink();
                _claw.setClawOpenTime(System.currentTimeMillis());
                _claw.clawOpen();
            }

            _claw.moveLift(_gamepad.left_stick_y);
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
