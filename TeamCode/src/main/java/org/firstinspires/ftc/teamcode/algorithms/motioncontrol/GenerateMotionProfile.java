package org.firstinspires.ftc.teamcode.algorithms.motioncontrol;

public class GenerateMotionProfile {
    ProfilePoint curPoint;
    double setPoint;
    double maxV;
    double maxA;
    double dt;

    public GenerateMotionProfile(double setPoint, double maxV, double maxA, double dt) {
        this.curPoint = new ProfilePoint(0,0,0,0);
        this.setPoint = setPoint;
        this.maxV = maxV;
        this.maxA = maxA;
        this.dt = dt;
    }
    public GenerateMotionProfile(ProfilePoint curPoint, double setPoint, double maxV, double maxA, double dt) {
        this.curPoint = curPoint;
        this.setPoint = setPoint;
        this.maxV = maxV;
        this.maxA = maxA;
        this.dt = dt;
    }

    public ProfilePoint[] generate(){


        ProfilePoint[] output = new ProfilePoint[requiredPoints()];
        // Generate first point
        output[0] = curPoint;
        int i = 1;
        // Acceleration
        for (; ; i++) {
            output[i] = new ProfilePoint(
                    dt + output[i-1].getTime(),
                    output[i-1].getPosition() + dt * output[i-1].getVelocity(),
                    output[i-1].getVelocity() + dt * maxA,
                    maxA);
            if (output[i].getVelocity() + dt * maxA - maxV > 0) break;
        }
        // Cruise
        for (;i < endOfCruise() ; i++) {
            output[i] = new ProfilePoint(
                    dt + output[i-1].getTime(),
                    output[i-1].getPosition() + dt * output[i-1].getVelocity(),
                    maxV,
                    0);
        }

        // Decceleration
        for (; ; i++) {
            output[i] = new ProfilePoint(
                    output[i-1].getTime() + dt,
                    output[i-1].getPosition() + dt * output[i-1].getVelocity(),
                    output[i-1].getVelocity() + dt * maxA,
                    -maxA);
            if (output[i].getVelocity() - dt * maxA > 0) break;
        }
        output[i+1] = new ProfilePoint(output[i].getTime() + dt,
                output[i].getPosition() + dt * output[i].getVelocity(),
                0,
                0);

        return output;

    }

    double distancetoaccelerate(){
        return curPoint.getVelocity() * timetoaccelerate() + 1 / 2 * (maxV - curPoint.getVelocity()) * timetoaccelerate();
    }

    double distancetoDecelerate(){
        return maxV * timetoDecelerate() * 1/ 2;
    }

    double timetoaccelerate(){
        return Math.abs((maxV - curPoint.getVelocity())/maxA)
                ;
    }

    double timetoDecelerate() {
        return maxV/maxA;
    }

    double totalTime(){

    }

    double endOfCruise(){
        return
    }
}
