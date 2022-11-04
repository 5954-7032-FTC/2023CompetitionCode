package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.algorithms.motorRampProfile;
import org.firstinspires.ftc.teamcode.threads.FieldPosition;
import org.firstinspires.ftc.teamcode.threads.RobotThread;


public class LinearMovement {

    DcMotor [] _DriveMotors;
    Telemetry.Item _T_FR, _T_RR, _T_FL, _Z_RL,_ZL,_ZF,_ZR, _RR,_T_RUNTO,_T_CURR;

    public LinearMovement(DcMotor[] motors, Telemetry telemetry) {
        //_drive.Initialize(motors,telemetry);
        _DriveMotors = motors;
        _DriveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        _DriveMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);


        _T_FR = telemetry.addData("FR", 0);
        _T_RR = telemetry.addData("RR", 0);
        _T_FL = telemetry.addData("FL", 0);
        _Z_RL = telemetry.addData("RL", 0);

        _T_RUNTO = telemetry.addData("RunTo", 0);
        _T_CURR = telemetry.addData("Curr", 0);
    }



    public void leftsetMode(DcMotor.RunMode run) {
        _DriveMotors[2].setMode(run);
        _DriveMotors[3].setMode(run);
    }
    public void rightsetMode(DcMotor.RunMode run) {
        _DriveMotors[0].setMode(run);
        _DriveMotors[1].setMode(run);
    }

    public void leftsetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        _DriveMotors[2].setZeroPowerBehavior(behavior);
        _DriveMotors[3].setZeroPowerBehavior(behavior);
    }

    public void rightsetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        _DriveMotors[0].setZeroPowerBehavior(behavior);
        _DriveMotors[1].setZeroPowerBehavior(behavior);
    }

    public int leftgetCurrentPosition() {
        return _DriveMotors[2].getCurrentPosition();
    }
    public int rightgetCurrentPosition() {
        return _DriveMotors[1].getCurrentPosition();
    }

    public void leftsetTargetPosition(int leftTarget) {
        _DriveMotors[3].setTargetPosition(leftTarget);
        _DriveMotors[2].setTargetPosition(leftTarget);
    }

    public void rightsetTargetPosition(int rightTarget) {
        _DriveMotors[0].setTargetPosition(rightTarget);
        _DriveMotors[1].setTargetPosition(rightTarget);
    }

    public boolean leftisBusy() {
        return _DriveMotors[2].isBusy();
    }

    public boolean rightisBusy() {
        return _DriveMotors[1].isBusy();
    }

    public void leftsetPower(double leftspeed) {
        _DriveMotors[3].setPower(leftspeed);
        _DriveMotors[2].setPower(leftspeed);
    }

    public void rightsetPower(double rightspeed) {
        _DriveMotors[0].setPower(rightspeed);
        _DriveMotors[1].setPower(rightspeed);
    }

    public void leftLateralsetTargetPosition(int leftTarget) {
        _DriveMotors[3].setTargetPosition(leftTarget);
        _DriveMotors[1].setTargetPosition(leftTarget);
    }
    public void rightLateralsetTargetPosition(int rightTarget) {
        _DriveMotors[0].setTargetPosition(rightTarget);
        _DriveMotors[2].setTargetPosition(rightTarget);
    }
    public int leftLateralgetCurrentPosition() {
        return _DriveMotors[3].getCurrentPosition();
    }
    public int rightLateralgetCurrentPosition() {
        return _DriveMotors[0].getCurrentPosition();
    }

    public void leftLateralsetPower(double leftspeed) {
        _DriveMotors[3].setPower(leftspeed);
        _DriveMotors[1].setPower(leftspeed);
    }

    public void rightLateralsetPower(double rightspeed) {
        _DriveMotors[0].setPower(rightspeed);
        _DriveMotors[2].setPower(rightspeed);
    }
}
