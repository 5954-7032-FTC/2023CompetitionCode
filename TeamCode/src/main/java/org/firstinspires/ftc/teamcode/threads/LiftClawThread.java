package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class LiftClawThread extends RobotThread {
    private final LiftClaw _liftclaw;
    private final Gamepad _gamepad;


    public LiftClawThread(DcMotor Motor, Servo [] servos, TouchSensor stop, DistanceSensor release,
                          Telemetry telemetry, Gamepad gamepad) {
        _liftclaw = new LiftClaw();
        _liftclaw.Initialize(Motor,servos,stop,release,telemetry);
        _gamepad=gamepad;
    }


    public void run() {
        while (!isCancelled()) {
            //lift movement
            _liftclaw.Move((double) _gamepad.left_stick_y);

            //  claw control
            if (_gamepad.right_trigger > 0) {
                _liftclaw.ClawOpen();
            } else {
                _liftclaw.ClawCloseThreaded();
            }


            if (_liftclaw.checkOptical() ) {
                long currentTimeMillis = System.currentTimeMillis();
                new Thread() {
                    @Override
                    public void run() {
                        while (currentTimeMillis + 1000 < System.currentTimeMillis() ) {
                            // do nothing
                            try {
                                Thread.sleep(10);
                            }
                            catch (InterruptedException ie) {

                            }
                        }
                        _liftclaw.ClawCloseThreaded();
                    }
                }.start();
            }

            if (_gamepad.x) {
                new Thread() {
                    @Override
                    public void run() {
                        _liftclaw.Move(LiftClaw.BOTTOM);
                    }
                }.start();
            }
            if (_gamepad.a) {
                new Thread() {
                    @Override
                    public void run() {
                        _liftclaw.Move(LiftClaw.LOW);
                    }
                }.start();
            }
            if (_gamepad.b) {
                new Thread() {
                    @Override
                    public void run() {
                        _liftclaw.Move(LiftClaw.MEDIUM);
                    }
                }.start();
            }
            if (_gamepad.y) {
                new Thread() {
                    @Override
                    public void run() {
                        _liftclaw.Move(LiftClaw.HIGH);
                    }
                }.start();
            }


        }


    }
}
