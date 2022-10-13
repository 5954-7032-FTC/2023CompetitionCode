package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ericw on 10/8/2017.
 */

public class IMU {

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles = new Orientation();
    Acceleration gravity = new Acceleration();
    Position location = new Position();
    Velocity speed = new Velocity();
    private boolean Enabled;
    private Thread T;

    public void Initialize(BNO055IMU Link){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = Link;
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Run IMU
        // TODO: 10/21/2017 Determine appropreate timing
        T = new Thread(new Runnable() {
            @Override
            public void run() {
                while(Enabled) {
                    ReadIMU();
                    try {Thread.sleep(10);
                    } catch (InterruptedException e){}
                }
            }
        });
    }

    public void ReadIMU(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        location = imu.getPosition();
        speed = imu.getVelocity();
    }

    public void Enable(){
        Enabled = true;
        T.start();
    }

    public void Disable(){
        Enabled = false;
    }

    public Orientation getAngles() {
        return angles;
    }

    public Acceleration getGravity() {
        return gravity;
    }

    public Position getLocation() {
        return location;
    }

    public Velocity getSpeed() {
        return speed;
    }


}