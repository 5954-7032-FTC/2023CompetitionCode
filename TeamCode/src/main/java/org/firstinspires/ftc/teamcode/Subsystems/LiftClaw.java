package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftClaw {

    DcMotor DriveM;
    ServoController[] clawServos;
    TouchSensor bottomstopSensor;
    OpticalDistanceSensor releaseSensor;
    Telemetry telemetry;

    final static int CLAW_OPEN = 0;
    final static int CLAW_CLOSE = 1;
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;

    final static int CALIBRATE = 0;
    public final static int BOTTOM = 1;
    public final static int LOW = 2;
    public final static int MEDIUM = 3;
    public final static int HIGH = 4;


    final static double BOTTOM_POS = 0;
    final static double LOW_POS = 100;
    final static double MEDIUM_POS = 200;
    final static double HIGH_POS = 300;


    /******************************************
     * Initialize with a motor array
     * @param Motor Define motors
     */
    public void Initialize(DcMotor Motor, Servo servos[], TouchSensor stop, OpticalDistanceSensor release, Telemetry telemetry) {
        DriveM = Motor;
        DriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServos = new ServoController[2];
        for (int i = 0; i < servos.length; i++) {
            clawServos[i] = servos[i].getController();
            clawServos[i].setServoPosition(0, i);
        }

        bottomstopSensor = stop;
        releaseSensor = release;

        this.telemetry = telemetry;
        Calibrate();
    }

    public void Calibrate() {
        int last = -1;
        int current = 0;

        int delta = 5;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        current = DriveM.getCurrentPosition();
        DriveM.setPower(-0.5);
        while (Math.hypot(current, last) > delta
                || !bottomstopSensor.isPressed()
        ) {
            last = current;
            current = DriveM.getCurrentPosition();
        }
        DriveM.setPower(0);
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    double releaseSensor_max = releaseSensor.getRawLightDetectedMax();
    double releaseSensor_max_threshold = 0.05; //threshold
    double releaseSensor_max_pct = releaseSensor_max * releaseSensor_max_threshold;

    public void checkOptical() {
        double current = releaseSensor.getLightDetected();
        double percent = 0.05; // percent as decimal
        if (Math.hypot(releaseSensor_max, current) < releaseSensor_max_pct) {
            ClawOpen();
        }
    }

    public double Move(int location) {
        switch (location) {
            case BOTTOM:
                DriveM.setTargetPosition(BOTTOM_POS);
                break;
            case LOW:
                DriveM.setTargetPosition(LOW_POS);
                break;
            case MEDIUM:
                DriveM.setTargetPosition(MEDIUM_POS);
                break;
            case HIGH:
                DriveM.setTargetPosition(HIGH_POS);
                break;
        }
        return 0;
    }

    public double Move(double move) {
        if (DriveM.getCurrentPosition() > 0) {
            DriveM.setPower(Range.clip(-move, -1, 1));
        } else {
            DriveM.setPower(0);
        }
        telemetry.addData("Position", (getEncoder()));
        telemetry.update();

        return move;
    }


    public void ClawOpen() {
        clawServos[CLAW_A].setServoPosition(0, 1);
        clawServos[CLAW_B].setServoPosition(1, 0);
    }

    public void ClawClose() {
        clawServos[CLAW_A].setServoPosition(0, 0);
        clawServos[CLAW_B].setServoPosition(1, 1);
    }

    private double deadzone(double power) {
        return Math.abs(power) > 0.1 ? power : 0.0;
    }

    public double getEncoder() {
        return DriveM.getCurrentPosition();
    }

    public final static double ENC_SCALE = Math.PI * 7 / (8 * 280);

    public double getPosition() {
        return getEncoder() * ENC_SCALE;
    }

    public boolean HasEncodersReset() {
        return getEncoder() == 0;
    }

    public void SetMode(DcMotor.RunMode SMode) {
        DriveM.setMode(SMode);
    }

}
