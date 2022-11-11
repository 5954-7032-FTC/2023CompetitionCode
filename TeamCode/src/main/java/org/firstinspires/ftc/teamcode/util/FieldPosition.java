package org.firstinspires.ftc.teamcode.util;

public class FieldPosition {

    public float X,Y,Z;
    public float roll,pitch,heading;

    public long lastupdatemillis;

    public FieldPosition(float X, float Y, float Z, float roll, float pitch, float heading) {
        this.X = X;
        this.Y = Y;
        this.Z = Z;
        this.roll = roll;
        this.pitch = pitch;
        this.heading = heading;
    }

    public void update(float X, float Y, float Z, float roll, float pitch, float heading) {
        this.X = X;
        this.Y = Y;
        this.Z = Z;
        this.roll = roll;
        this.pitch = pitch;
        this.heading = heading;
        this.lastupdatemillis = System.currentTimeMillis();
    }

    public float getX() {
        return X;
    }

    public void setX(float x) {
        X = x;
    }

    public float getY() {
        return Y;
    }

    public void setY(float y) {
        Y = y;
    }

    public float getZ() {
        return Z;
    }

    public void setZ(float z) {
        Z = z;
    }

    public float getRoll() {
        return roll;
    }

    public void setRoll(float roll) {
        this.roll = roll;
    }

    public float getPitch() {
        return pitch;
    }

    public void setPitch(float pitch) {
        this.pitch = pitch;
    }

    public float getHeading() {
        return heading;
    }

    public void setHeading(float heading) {
        this.heading = heading;
    }
}
