package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MecanumDrive {
    public static class Parameters {
        public double     COUNTS_PER_MOTOR_REV    = 1120 ;   // 1120 per revolution
        public double     DRIVE_GEAR_REDUCTION    = 24.0/32.0; //   3/4
        public double     WHEEL_DIAMETER_INCHES   = 96/25.4 ;     // For figuring circumference
        public double  COUNTS_PER_INCH_FORWARD = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        public double ROBOT_DIAMETER_IN = 13;
        public double COUNTS_PER_ROTATE = (ROBOT_DIAMETER_IN * Math.PI)*COUNTS_PER_INCH_FORWARD;
        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        public double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
        public double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate

        public int [] _FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
        public int [] _ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
        public int [] _REVERSED_WHEELS; // reversed motors.
        public DcMotor [] motors;
        public boolean robotCentric = true;
        public Telemetry telemetry;
        public double _SPEED_FACTOR =1.4;
        public double _ROTATION_RATE =0.75;
        //motor directions RF, RR, LR, LF, rotate
        public int [] FORWARD_VALUES = { 1, 1, 1, 1,1};
        public int [] REVERSE_VALUES = {-1, -1, -1, -1,-1};
        public int [] LEFT_VALUES = {1,-1,1,-1,1};
        public int [] RIGHT_VALUES = {-1,1,-1,1,1};
        public int [] ROTATE_VALUES = {1,1,-1,-1};
    }

    protected double     COUNTS_PER_MOTOR_REV;   // 1120 per revolution
    protected double     DRIVE_GEAR_REDUCTION; //   3/4
    protected double     WHEEL_DIAMETER_INCHES;     // For figuring circumference
    protected double     COUNTS_PER_INCH_FORWARD;

    protected double ROBOT_DIAMETER;
    protected double COUNTS_PER_ROTATE;
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    protected double     DRIVE_SPEED;     // Max driving speed for better distance accuracy.
    protected double     TURN_SPEED;     // Max Turn speed to limit turn rate
    protected int [] _FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
    protected int [] _ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
    protected int [] _REVERSED_WHEELS; // reversed motors.
    protected DcMotor [] motors;
    protected double _SPEED_FACTOR;
    protected double _ROTATION_RATE;
    protected Telemetry _telemetry;
    int [] FORWARD_VALUES, REVERSE_VALUES, LEFT_VALUES, RIGHT_VALUES, ROTATE_VALUES;


    public MecanumDrive(Parameters parameters) {
        this.COUNTS_PER_MOTOR_REV = parameters.COUNTS_PER_MOTOR_REV;
        this.DRIVE_GEAR_REDUCTION = parameters.DRIVE_GEAR_REDUCTION;
        this.WHEEL_DIAMETER_INCHES = parameters.WHEEL_DIAMETER_INCHES;
        this.COUNTS_PER_INCH_FORWARD = parameters.COUNTS_PER_INCH_FORWARD;
        this.ROBOT_DIAMETER = parameters.ROBOT_DIAMETER_IN;
        this.COUNTS_PER_ROTATE = parameters.COUNTS_PER_ROTATE;
        this.DRIVE_SPEED = parameters.DRIVE_SPEED;
        this.TURN_SPEED = parameters.TURN_SPEED;
        this._FREE_WHEELS = parameters._FREE_WHEELS;
        this._ENCODER_WHEELS = parameters._ENCODER_WHEELS;
        this._REVERSED_WHEELS = parameters._REVERSED_WHEELS;
        this._ROTATION_RATE = parameters._ROTATION_RATE;
        this._SPEED_FACTOR = parameters._SPEED_FACTOR;
        this.motors = parameters.motors;
        this._telemetry = parameters.telemetry;
        this.FORWARD_VALUES = parameters.FORWARD_VALUES;
        this.LEFT_VALUES = parameters.LEFT_VALUES;
        this.RIGHT_VALUES = parameters.RIGHT_VALUES;
        this.REVERSE_VALUES = parameters.REVERSE_VALUES;
        this.ROTATE_VALUES = parameters.ROTATE_VALUES;
        initMecanum();

    }

    // constants
    protected static final double SQRT2=Math.sqrt(2);
    protected static final double SQRT2_OVER2 = SQRT2/2;
    protected static final double PI_OVER4=Math.PI/4;

    public boolean bendSinisterIsBusy() {
        return (motors[0].isBusy() || motors[2].isBusy());
    }

    public boolean bendDexterIsBusy() {  // return true if either is busy.
        return (motors[1].isBusy() || motors[3].isBusy());
    }

    public void driveForward(double distance) {
        driveRobot(distance, FORWARD_VALUES);
    }

    public void driveLeft(double distance) {
        driveRobot(distance, LEFT_VALUES);
    }

    public void driveReverse(double distance) {
        driveRobot(distance, REVERSE_VALUES);
    }

    public void driveRight(double distance) {
        driveRobot(distance, RIGHT_VALUES);
    }

    public void driveRobot(double inches, int [] direction) {
        int moveCounts = moveCounts(inches);
        setTargetPositions(moveCounts, direction);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
        moveRobotDirection(DRIVE_SPEED, 0 ,direction);
        // keep looping while we are still active, and BOTH motors are running.
        while (true)
            if ( ! (bendDexterIsBusy() && bendSinisterIsBusy() ) ) {
                // Stop all motion & Turn off RUN_TO_POSITION
                moveRobotDirection(0, 0, FORWARD_VALUES);
                setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
    }

    public double getRotationRate() {
        return _ROTATION_RATE;
    }

    public double getSpeedFactor() {
        return _SPEED_FACTOR;
    }

    protected void initMecanum() {
        setRunMode(_FREE_WHEELS, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(_REVERSED_WHEELS, DcMotorSimple.Direction.REVERSE);
        setZeroPowerBehavior(new int[]{0,1,2,3},DcMotor.ZeroPowerBehavior.BRAKE);
        //COUNTS_PER_INCH_FORWARD         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    }

    public void movePolar(double power, double angle, double rotate) {
        angle -= PI_OVER4;
        rotate *= _ROTATION_RATE;
        double sine  = Math.sin(angle);
        double cosine = Math.cos(angle);
        double scale = ( (power + Math.abs(rotate)) > 1 ) ? _SPEED_FACTOR /(power + rotate) : _SPEED_FACTOR * SQRT2_OVER2  ;

        double [] wheelSpeeds = {
                scale * (power * sine - rotate),   // Front Right
                scale * (power * cosine - rotate), // Rear Rights
                scale * (power * sine + rotate),   // Rear Left
                scale * (power * cosine + rotate)  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
    }

    public void moveRect(double forward, double lateral, double rotate) {
        //translate into polar and move accordingly.
        movePolar(Math.hypot(forward,lateral),
                Math.atan2(-forward,lateral),
                rotate);
    }

    public int moveCounts(double distance) {
        return (int)(distance * COUNTS_PER_INCH_FORWARD);
    }

    public void moveRobotDirection(double power, double rotate, int [] direction) {
        rotate *= direction[4];
        double [] wheelSpeeds = {
                direction[0]*power - rotate,   // Front Right
                direction[1]*power - rotate, // Rear Right
                direction[2]*power + rotate,   // Rear Left
                direction[3]*power + rotate  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        _telemetry.update();
    }

    public int [] readEncoders() {  // return the encoder values if there are encoder motors.
        if (_ENCODER_WHEELS != null && _ENCODER_WHEELS.length > 0 ) {
            int [] encoders = new int[_ENCODER_WHEELS.length];
            for (int i=0; i<_ENCODER_WHEELS.length; i++) {
                encoders[i] = motors[_ENCODER_WHEELS[i]].getCurrentPosition();
            }
            return encoders;
        }
        return null;
    }

    public int rotateCounts(double degrees) {
        return (int)Math.round((degrees * COUNTS_PER_ROTATE)/360);
    }

    public void setDirection(int [] wheels, DcMotorSimple.Direction dir) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                motors[wheel].setDirection(dir);

    }

    public void setMotorSpeeds(double [] speeds) {
        for (int i=0; i< motors.length; i++) motors[i].setPower(Range.clip(speeds[i], -1, 1));
    }

    public void setRotationRate(double RotationRate) {
        this._ROTATION_RATE = RotationRate;
    }

    public void setRunMode(int [] wheels, DcMotor.RunMode mode) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                motors[wheel].setMode(mode);
    }

    public void setTargetPositions(int target, int [] directions) {
        for (int i=0; i<_ENCODER_WHEELS.length; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + target * directions[i]);
        }
    }

    public void setSpeedFactor(double SpeedFactor) {
        this._SPEED_FACTOR = SpeedFactor;
    }

    public void setZeroPowerBehavior( int [] wheels, DcMotor.ZeroPowerBehavior behavior) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                motors[wheel].setZeroPowerBehavior(behavior);
    }


    // CCW == positive degrees
    // CW == negative degrees
    public void turnRobot(double degrees) {
        int moveCounts = rotateCounts(degrees);
        setTargetPositions(moveCounts, ROTATE_VALUES);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
        moveRobotDirection(DRIVE_SPEED, 0, ROTATE_VALUES);
        while (true) {
            if (!(bendDexterIsBusy() && bendSinisterIsBusy())) {
                moveRobotDirection(0, 0, FORWARD_VALUES);
                setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
    }

}
