package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MechanumDrive;


public class MovementThread extends RobotThread {
    private final Gamepad _gamepad;
    private final MechanumDrive _drive = new MechanumDrive();

    public MovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry) {
        this._gamepad = gamepad;
        _drive.Initialize(motors,telemetry);

    }

    public void run() {
        while (!isCancelled()) {

            //lateral
            //forward
            //rotate
            //each has a deadzone and ramprate

            if (_gamepad.dpad_left) {
                _drive.setZoneLateral( _drive.getZoneLateral() - 0.02);
            }
            if (_gamepad.dpad_right) {
                _drive.setZoneLateral( _drive.getZoneLateral() + 0.02);
            }
            if (_gamepad.dpad_up) {
                _drive.setZoneForward( _drive.getZoneForward() + 0.02);
            }
            if (_gamepad.dpad_down) {
                _drive.setZoneForward( _drive.getZoneForward() - 0.02);
            }
            if (_gamepad.right_bumper) {
                _drive.setZoneRotate( _drive.getZoneRotate() - 0.02 );
            }
            if (_gamepad.right_trigger > 0.5 ) {
                _drive.setZoneRotate( _drive.getZoneRotate() + 0.02 );
            }
            if (_gamepad.left_bumper) {
                _drive.setRampRate( _drive.getRampRate() + 0.02 );
            }
            if (_gamepad.left_trigger > 0.5 ) {
                _drive.setRampRate( _drive.getRampRate() - 0.02 );
            }


            _drive.Drive(_gamepad.left_stick_y,_gamepad.left_stick_x,_gamepad.right_stick_x);

        }
    }


}
