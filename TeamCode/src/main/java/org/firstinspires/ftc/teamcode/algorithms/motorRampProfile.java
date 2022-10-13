package org.firstinspires.ftc.teamcode.algorithms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class motorRampProfile {
    ElapsedTime rampTimer = new ElapsedTime();
    double curPoint = 0;
    double prevT = 0;
    double prevSign = 0;
    double rampRate;

    public motorRampProfile(double rampRate){
        this.rampRate = rampRate;
    }

    public double ramp(double input, double rampRate){
        this.rampRate = rampRate;
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double nextPoint = (Math.abs(curPoint) + this.rampRate * (curTime - prevT));
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
    public double ramp(double input){
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double nextPoint = (Math.abs(curPoint) + this.rampRate * (curTime - prevT));
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

}
