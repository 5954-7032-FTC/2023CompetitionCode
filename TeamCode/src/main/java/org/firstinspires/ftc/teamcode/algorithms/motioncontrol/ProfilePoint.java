package org.firstinspires.ftc.teamcode.algorithms.motioncontrol;

public class ProfilePoint {
    private double time, position, velocity, acceleration;

    /***
     * Profile Point, creates an individual profile point for motion profiling
     * @param time Time in seconds
     * @param position Position in Inches
     * @param velocity Velocity in Inches per second
     * @param acceleration Acceleration in Inches per Second^2
     */
    public ProfilePoint(double time, double position, double velocity, double acceleration) {
        this.time = time;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    /***
     * Get Acceleration
     * @return Acceleration in Inches per Second^2
     */
    public double getAcceleration() {
        return acceleration;
    }

    /**
     * Get Position
     * @return Position in Inches
     */
    public double getPosition() {
        return position;
    }

    /**
     * Get Time
     * @return Time in Seconds
     */
    public double getTime() {
        return time;
    }

    /**
     * Get Velocity
     * @return Velocity in Inches per Second
     */
    public double getVelocity() {
        return velocity;
    }
}
