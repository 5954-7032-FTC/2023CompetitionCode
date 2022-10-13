package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MechanumDrive {

    DcMotor DriveM[];
    private final double OUTPUT_SCALE_FACTOR = 1.0;

    /******************************************
     * Initialize with a motor array
     * @param Motors Front -> Rear, Left -> Right in order of initialization
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


        //normalize(wheelSpeeds);
        //scale(wheelSpeeds, OUTPUT_SCALE_FACTOR);

        for (int i = 0; i < 4; i++) {
            DriveM[i].setPower(Range.clip(wheelSpeeds[i],-1,1));
        }

        return wheelSpeeds;
    }

    private double deadzone(double power) {
        return Math.abs(power) > 0.1 ? power : 0.0 ;
    }

    private static void scale(double wheelSpeeds[], double scaleFactor) {
        for (double x :
                wheelSpeeds) {
            x *= scaleFactor;
        }
    }

    private static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (double p :
                wheelSpeeds) {
            maxMagnitude = Math.max(maxMagnitude, p);
        }

        if (maxMagnitude > 1.0) {
            for (double q :
                    wheelSpeeds) {
                q = q / maxMagnitude;
            }
        }
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

    /*******************
     * Ramp Attempt
     */

    ElapsedTime rampTimer = new ElapsedTime();
    double curPoint = 0;
    double prevT = 0;
    double prevSign = 0;
    public double rampP(double input, double rampRate){
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double nextPoint = (Math.abs(curPoint) + rampRate * (curTime - prevT));
        if (prevSign != curSign && prevSign !=0){
            curPoint = 0;
        }
        else if (Math.abs(input) - Math.abs(nextPoint) > 0){
            curPoint = curSign * nextPoint;
        }
        else{
            curPoint = input;
        }
        prevSign = curSign;
        prevT = curTime;
        return curPoint;
    }

    double curPoint1 = 0;
    double prevSign1 = 0;
    double prevT1 = 0;
    public double rampP1(double input, double rampRate){
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double nextPoint = (Math.abs(curPoint1) + rampRate * (curTime - prevT1));
        if (prevSign1 != curSign && prevSign1 !=0){
            curPoint1 = 0;
        }
        else if (Math.abs(input) - Math.abs(nextPoint) > 0){
            curPoint1 = curSign * nextPoint;
        }
        else{
            curPoint1 = input;
        }
        prevSign1 = curSign;
        prevT1 = curTime;
        return curPoint1;
    }

    double curPoint2 = 0;
    double prevSign2 = 0;
    double prevT2 = 0;
    public double rampP2(double input, double rampRate){
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double nextPoint = (Math.abs(curPoint2) + rampRate * (curTime - prevT2));
        if (prevSign2 != curSign && prevSign2 !=0){
            curPoint2 = 0;
        }
        else if (Math.abs(input) - Math.abs(nextPoint) > 0){
            curPoint2 = curSign * nextPoint;
        }
        else{
            curPoint2 = input;
        }
        prevSign2 = curSign;
        prevT2 = curTime;
        return curPoint2;
    }

}
