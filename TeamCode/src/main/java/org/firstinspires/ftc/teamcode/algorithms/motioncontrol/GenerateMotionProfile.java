package org.firstinspires.ftc.teamcode.algorithms.motioncontrol;

import java.util.LinkedList;
import java.util.Queue;

public class GenerateMotionProfile {

    public static Queue<ProfilePoint> acceleration(ProfilePoint LastPoint, double vel, double accel, double dt){

        Queue<ProfilePoint> output = new LinkedList<>();

        for (double t = LastPoint.getTime(); t < timetoaccelerate(LastPoint,vel,accel) ; t=+dt) {

            output.add( new ProfilePoint(
                    t,
                    LastPoint.getPosition() + LastPoint.getVelocity()*(t-LastPoint.getTime()) + 0.5 * accel * Math.pow(t-LastPoint.getTime(),2),
                    (vel - LastPoint.getVelocity() > 0 ? 1 : -1) * accel * (t-LastPoint.getTime()),
                    accel));
        }

        return output;
    }

    public static Queue<ProfilePoint> cruse(ProfilePoint LastPoint, double time, double dt){

        Queue<ProfilePoint> output = new LinkedList<>();

        for (double t = LastPoint.getTime(); t < time - LastPoint.getTime() ; t=+dt) {

            output.add(new ProfilePoint(
                    t,
                    LastPoint.getPosition() + LastPoint.getVelocity()*(t-LastPoint.getTime()),
                    LastPoint.getVelocity(),
                    0));
        }

        return output;
    }


    static double timetoaccelerate(ProfilePoint curPoint, double vel, double accel){
        return Math.abs((vel - curPoint.getVelocity())/accel);
    }

}
