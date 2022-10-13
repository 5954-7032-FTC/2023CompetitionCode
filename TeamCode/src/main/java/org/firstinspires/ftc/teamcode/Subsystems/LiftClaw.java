package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftClaw {

    DcMotor DriveM;
    ServoController clawServo;
    RevTouchSensor bottomstopSensor;
    RevColorSensorV3 releaseSensor;
    Telemetry telemetry;

    final static int CLAW_OPEN=0;
    final static int CLAW_CLOSED=1;

    final static int CALIBRATE=0;
    final static int BOTTOM=1;
    final static int LOW=2;
    final static int MEDIUM=3;
    final static int HIGH=4;


    final static double BOTTOM_POS=0;
    final static double LOW_POS=100;
    final static double MEDIUM_POS=200;
    final static double HIGH_POS=300;





    /******************************************
     * Initialize with a motor array
     * @param Motor Define motors
     */
    public void Initialize(DcMotor Motor, CRServo servo, RevTouchSensor stop, RevColorSensorV3 release, Telemetry telemetry){
        DriveM = Motor;
        DriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServo = servo.getController();
        bottomstopSensor = stop;
        releaseSensor = release;
        this.telemetry=telemetry;
        Calibrate();
    }

    public void Calibrate() {
        while ( ! bottomstopSensor.isPressed() ) {
            DriveM.setPower(-1);
        }
        DriveM.setPower(0);
        DriveM.setPower(.05);
        try {
            Thread.sleep(100);
        }
        catch (Exception e) {

        }
        while ( ! bottomstopSensor.isPressed() ) {
            DriveM.setPower(-0.1);
        }
        DriveM.setPower(0);
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double Move(int location) {
        switch (location) {
            case BOTTOM:

                break;
            case LOW:
                break;
            case MEDIUM:
                break;
            case HIGH:
                break;
        }
        return 0;
    }

    public double Move(double move, int clawstate) {
        DriveM.setPower(Range.clip(move,-1,1));
        clawServo.setServoPosition(0, clawstate);

        telemetry.addData("Position", (getEncoder()));
        telemetry.update();

        return move;
    }

    private double deadzone(double power) {
        return Math.abs(power) > 0.1 ? power : 0.0 ;
    }

    public double getEncoder(){
        return DriveM.getCurrentPosition();
    }

    public final static double ENC_SCALE = Math.PI * 7 / ( 8 * 280);
    public double getPosition() {
        return getEncoder() * ENC_SCALE;
    }

    public boolean HasEncodersReset(){
        return getEncoder() == 0;
    }

    public void SetMode(DcMotor.RunMode SMode){
        DriveM.setMode(SMode);
    }

}
