package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MechanumDrive {

    DcMotor DriveM[];

    /******************************************
     * Initialize with a motor array
     * @param Motors Define motors in CW order starting with FR
     */
    public void Initialize(DcMotor[] Motors){
        DriveM = Motors;
        DriveM[2].setDirection(DcMotorSimple.Direction.REVERSE);
        DriveM[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double[] Drive(double ForwardPower, double LateralPower, double RotationalPower){


        double x = deadzone(LateralPower);
        double y = deadzone(ForwardPower);
        double r = deadzone(RotationalPower);

        double angle = Math.atan2(-y, x);
        double power = Math.hypot(y, x);

        double sine = Math.sin(angle-Math.PI/4);
        double cosine = Math.cos(angle-Math.PI/4);
        double scale = ( (power + Math.abs(r)) > 1 ) ? 1/(power + r) : 1 ;

        double wheelSpeeds[] = {
                scale * (power * sine - r),
                scale * (power * cosine - r),
                scale * (power * sine + r),
                scale * (power * cosine + r)
        };

        for (int i = 0; i < 4; i++) {
            DriveM[i].setPower(Range.clip(wheelSpeeds[i],-1,1));
        }

        return wheelSpeeds;
    }

    private double deadzone(double power) {
        return Math.abs(power) > 0.1 ? power : 0.0 ;
    }

    public double getEncoders(){
        return DriveM[3].getCurrentPosition();
    }

    public final static double ENC_SCALE = Math.PI * 7 / ( 8 * 280);
    public double getPosition(){
        return getEncoders() * ENC_SCALE;
    }

    public boolean HasEncodersReset(){
        return getEncoders() == 0;
    }

    public void SetMode(DcMotor.RunMode SMode){
        for (DcMotor i:
             DriveM) {
            i.setMode(SMode);
        }
    }

}
