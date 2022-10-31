package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.algorithms.motorRampProfile;


public class MovementThread extends RobotThread {
    private final Gamepad _gamepad;
    double speed_factor=1.4;

    double _zone_lateral = 0.1;
    double _zone_forward = 0.1;
    double _zone_rotation =0.1;
    double _ramp_rate = 1.5;

    DcMotor [] _DriveMotors;
    motorRampProfile _Joy1X, _Joy1Y, _Joy2X;
    Telemetry.Item _T_FR, _T_RR, _T_FL, _Z_RL,_ZL,_ZF,_ZR, _RR;

    public MovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry) {
        this._gamepad = gamepad;
        //_drive.Initialize(motors,telemetry);
        _DriveMotors = motors;
        _DriveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        _DriveMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        _Joy1Y = new motorRampProfile(_ramp_rate);
        _Joy1X = new motorRampProfile(_ramp_rate);
        _Joy2X = new motorRampProfile(_ramp_rate);


        _T_FR = telemetry.addData("FR", 0);
        _T_RR = telemetry.addData("RR", 0);
        _T_FL = telemetry.addData("FL", 0);
        _Z_RL = telemetry.addData("RL", 0);
        /*
        _ZL = telemetry.addData("ZL", _zone_lateral);
        _ZF = telemetry.addData("ZF", _zone_forward);
        _ZR = telemetry.addData("ZR", _zone_rotation);
        _RR = telemetry.addData("RR", _ramp_rate);
        */
    }

    public void run() {
        while (!isCancelled()) {
            //lateral
            //forward
            //rotate
            //each has a deadzone and ramprate


        /*
            if (_gamepad.dpad_left) {
                setZoneLateral( getZoneLateral() - 0.02);
            }
            if (_gamepad.dpad_right) {
                setZoneLateral( getZoneLateral() + 0.02);
            }
            if (_gamepad.dpad_up) {
                setZoneForward( getZoneForward() + 0.02);
            }
            if (_gamepad.dpad_down) {
                setZoneForward( getZoneForward() - 0.02);
            }
            if (_gamepad.right_bumper) {
                setZoneRotate( getZoneRotate() - 0.02 );
            }
            if (_gamepad.right_trigger > 0.5 ) {
                setZoneRotate( getZoneRotate() + 0.02 );
            }
            if (_gamepad.left_bumper) {
                setRampRate( getRampRate() + 0.02 );
            }
            if (_gamepad.left_trigger > 0.5 ) {
                setRampRate( getRampRate() - 0.02 );
            }

         */
            Drive(_gamepad.left_stick_y,_gamepad.left_stick_x,_gamepad.right_stick_x);
        }
    }



    long _lateral_debounce =0;
    public double getZoneLateral() {
        return _zone_lateral;
    }
    public void setZoneLateral(double lateral_amount) {
        if (System.currentTimeMillis() > _lateral_debounce +500 ) {
            _lateral_debounce = System.currentTimeMillis();
            _zone_lateral = lateral_amount;
            _ZL.setValue(lateral_amount);
        }
    }

    long _zone_forward_debounce = 0;
    public double getZoneForward() {
        return _zone_forward;
    }
    public void setZoneForward(double forward_amount) {
        if (System.currentTimeMillis() > _zone_forward_debounce +500 ) {
            _zone_forward_debounce = System.currentTimeMillis();
            _zone_forward = forward_amount;
            _ZF.setValue(forward_amount);
        }
    }

    long _zone_rotate_debounce = 0;
    public double getZoneRotate() {
        return _zone_rotation;
    }
    public void setZoneRotate(double rotation_amount) {
        if (System.currentTimeMillis() > _zone_rotate_debounce +500 ) {
            _zone_rotate_debounce = System.currentTimeMillis();
            _zone_rotation = rotation_amount;
            _ZR.setValue(rotation_amount);
        }
    }

    long _ramp_rate_debounce =0;
    public double getRampRate() { return _ramp_rate; }
    public void setRampRate(double rampRate) {
        if (System.currentTimeMillis() > _ramp_rate_debounce +500 ) {
            _ramp_rate_debounce = System.currentTimeMillis();
            _ramp_rate = rampRate;
            _RR.setValue(rampRate);
        }
    }

    public void DriveTo(FieldPosition pos) {
        //calculate the correct heading from _pos to pos.....
        double angle = Math.atan2( _pos.Y-pos.Y, _pos.X-pos.X );

    }

    FieldPosition _pos;
    public void setCurrentFieldPosition(FieldPosition pos) {
        this._pos = pos;
    }


    public void Drive(double ForwardPower, double LateralPower, double RotationalPower){
        double lateral = _Joy1X.ramp(deadzone(LateralPower,_zone_lateral));
        double forward = _Joy1Y.ramp(deadzone(ForwardPower,_zone_forward));
        double rotate = _Joy2X.ramp(deadzone(RotationalPower,_zone_rotation)) * .75;


        if (_gamepad.right_trigger > 0.2) {
             lateral = 0.4* LateralPower;
             forward = 0.4* ForwardPower;
             rotate = 0.4* RotationalPower* .75;
        }

        double angle = Math.atan2(-forward, lateral);
        double power = Math.hypot(forward, lateral);

        double sine = Math.sin(angle-Math.PI/4);
        double cosine = Math.cos(angle-Math.PI/4);
        double scale = ( (power + Math.abs(rotate)) > 1 ) ? speed_factor/(power + rotate) : speed_factor/Math.sqrt(2) ;

        double [] wheelSpeeds = {
                scale * (power * sine - rotate),
                scale * (power * cosine - rotate),
                scale * (power * sine + rotate),
                scale * (power * cosine + rotate)
        };

        for (int i = 0; i < 4; i++) {
            _DriveMotors[i].setPower(Range.clip(wheelSpeeds[i],-1,1));
        }

        _T_FR.setValue(Range.clip(wheelSpeeds[0],-1,1));
        _T_RR.setValue(Range.clip(wheelSpeeds[1],-1,1));
        _Z_RL.setValue(Range.clip(wheelSpeeds[2],-1,1));
        _T_FL.setValue(Range.clip(wheelSpeeds[3],-1,1));
    }

    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }

}
