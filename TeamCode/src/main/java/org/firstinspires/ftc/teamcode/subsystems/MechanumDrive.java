package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.algorithms.*;

public class MechanumDrive {

    DcMotor DriveM[];
    double _zone_lateral = 0.1;
    double _zone_forward = 0.1;
    double _zone_rotation =0.1;
    double _ramp_rate = 1.5;
    motorRampProfile
            Joy1X,Joy1Y,Joy2X;
    private Telemetry.Item _T_FR, _T_RR, _T_FL, _Z_RL,_ZL,_ZF,_ZR, _RR;
    /******************************************
     * Initialize with a motor array
     * @param Motors Define motors in CW order starting with FR
     */
    public void Initialize(DcMotor[] Motors, Telemetry telemetry){
        DriveM = Motors;
        DriveM[2].setDirection(DcMotorSimple.Direction.REVERSE);
        DriveM[3].setDirection(DcMotorSimple.Direction.REVERSE);
        Joy1Y = new motorRampProfile(_ramp_rate);
        Joy1X = new motorRampProfile(_ramp_rate);
        Joy2X = new motorRampProfile(_ramp_rate);
        _T_FR = telemetry.addData("FR", 0);
        _T_RR = telemetry.addData("RR", 0);
        _T_FL = telemetry.addData("FL", 0);
        _Z_RL = telemetry.addData("RL", 0);
        _ZL = telemetry.addData("ZL", _zone_lateral);
        _ZF = telemetry.addData("ZF", _zone_forward);
        _ZR = telemetry.addData("ZR", _zone_rotation);
        _RR = telemetry.addData("RR", _ramp_rate);
    }

    public double getZoneLateral() {
        return _zone_lateral;
    }
    public void setZoneLateral(double lateral_amount) {
        _zone_lateral = lateral_amount;
        _ZL.setValue(lateral_amount);
    }

    public double getZoneForward() {
        return _zone_forward;
    }
    public void setZoneForward(double forward_amount) {
        _zone_forward = forward_amount;
        _ZF.setValue(forward_amount);
    }

    public double getZoneRotate() {
        return _zone_rotation;
    }
    public void setZoneRotate(double rotation_amount) {
        _zone_rotation =rotation_amount;
        _ZR.setValue(rotation_amount);
    }

    public void setRampRate(double rampRate) {
        _ramp_rate = rampRate;
        _RR.setValue(rampRate);
    }
    public double getRampRate() { return _ramp_rate; }


    double speed_factor=1.4;

    public double[] Drive(double ForwardPower, double LateralPower, double RotationalPower){


        double lateral = Joy1X.ramp(deadzone(LateralPower,_zone_lateral));
        double forward = Joy1Y.ramp(deadzone(ForwardPower,_zone_forward));
        double rotate = Joy2X.ramp(deadzone(RotationalPower,_zone_rotation));

        double angle = Math.atan2(-forward, lateral);
        double power = Math.hypot(forward, lateral);

        double sine = Math.sin(angle-Math.PI/4);
        double cosine = Math.cos(angle-Math.PI/4);
        double scale = ( (power + Math.abs(rotate)) > 1 ) ? speed_factor/(power + rotate) : speed_factor/Math.sqrt(2) ;

        double wheelSpeeds[] = {
                scale * (power * sine - rotate),
                scale * (power * cosine - rotate),
                scale * (power * sine + rotate),
                scale * (power * cosine + rotate)
        };

        for (int i = 0; i < 4; i++) {
            DriveM[i].setPower(Range.clip(wheelSpeeds[i],-1,1));
        }

        _T_FR.setValue(Range.clip(wheelSpeeds[0],-1,1));
        _T_RR.setValue(Range.clip(wheelSpeeds[1],-1,1));
        _Z_RL.setValue(Range.clip(wheelSpeeds[2],-1,1));
        _T_FL.setValue(Range.clip(wheelSpeeds[3],-1,1));
        return wheelSpeeds;
    }

    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }

}
