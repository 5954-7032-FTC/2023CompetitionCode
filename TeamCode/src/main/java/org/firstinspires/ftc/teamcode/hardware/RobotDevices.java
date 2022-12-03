package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.checkerframework.checker.units.qual.A;


public class RobotDevices {
    //public LineFollowerSensors lineFollowers;
    public ColorSensor colorSensorLeft,colorSensorRight;
    public DcMotor [] wheels;
    public Servo[] lift_servos;
    public BNO055IMU imu;
    public DcMotor lift_motor;
    public TouchSensor bottom_stop;
    public DistanceSensorDevice post_sensor;//,bottom_cone;
    //public Servo pipe_guide;
    //public Servo arm_release;
    public ArmRelease arm_release;

    //public NormalizedColorSensor [] lineFollowers;

    public ColorSensor cone_detector;

    protected static RobotDevices robot;

    public static RobotDevices getDevices(HardwareMap hardwareMap) {
        if (robot == null ) {
            robot = new RobotDevices(hardwareMap);
        }
        return robot;
    }

    private RobotDevices(HardwareMap hardwareMap) {
        //cone_detector = hardwareMap.colorSensor.get("CONE");

        /*
        lineFollowers = new LineFollowerSensors(new NormalizedColorSensor[] {
                hardwareMap.get(NormalizedColorSensor.class,"L_LINE"),
                hardwareMap.get(NormalizedColorSensor.class,"R_LINE")
        });

        */

        colorSensorRight = hardwareMap.colorSensor.get("RIGHT_COLOR");
        colorSensorLeft = hardwareMap.colorSensor.get("LEFT_COLOR");
        wheels = new DcMotor[]{
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")
        };
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        lift_motor = hardwareMap.dcMotor.get("LIFT");
        lift_servos = new Servo[]{
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };
        bottom_stop = hardwareMap.touchSensor.get("BSTOP");
        post_sensor = new DistanceSensorDevice(hardwareMap.get(DistanceSensor.class, "C_STOP"));
        //bottom_cone = new DistanceSensorDevice(hardwareMap.get(DistanceSensor.class, "BOT_CONE"));
        //pipe_guide = hardwareMap.servo.get("PIPE_GUIDE");
        arm_release = new ArmRelease(hardwareMap.servo.get("ARM_RELEASE"));
    }
}
