package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.algorithms.*;


@TeleOp(name = "TeleOp")

public class Teleop extends LinearOpMode {

    final static MechanumDrive drive = new MechanumDrive();
    final static LiftClaw lift = new LiftClaw();
    final static motorRampProfile Joy1Y = new motorRampProfile(1.5), Joy1X = new motorRampProfile(1.5), Joy2X = new motorRampProfile(1.5);
    int p = 0;


    int toNum(boolean in) {
        return (in ? 1 : 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        this.Initialize();
        telemetry.addData("Robot Ready", "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //movement
            drive.Drive(Joy1Y.ramp(gamepad1.left_stick_y, 1.5), Joy1X.ramp(gamepad1.left_stick_x, 1.5), Joy2X.ramp(gamepad1.right_stick_x, 1.5));

            //lift movement
            lift.Move((double) gamepad2.left_stick_y);

            //presets for height
            if (gamepad2.x) {
                lift.Move(lift.BOTTOM);
            }
            if (gamepad2.a) {
                lift.Move(lift.LOW);
            }
            if (gamepad2.b) {
                lift.Move(lift.MEDIUM);
            }
            if (gamepad2.y) {
                lift.Move(lift.HIGH);
            }

            //  claw control
            if (gamepad2.right_trigger > 0) {
                lift.ClawOpen();
            } else {
                lift.ClawClose();
            }

            lift.checkOptical();

            lift.testBottom();

            idle();
        }

    }


    public void Initialize() {

        //release arm servo
        hardwareMap.servo.get("ARM_RELEASE").setPosition(.5);

        final DcMotor init_drive[] = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};

        drive.Initialize(init_drive);

        final DcMotor init_lift = hardwareMap.dcMotor.get("LIFT");
        final Servo servos[] = {
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };


        lift.Initialize(init_lift,
                servos,
                hardwareMap.touchSensor.get("BSTOP"),
                //hardwareMap.opticalDistanceSensor.get("C_STOP"),
                hardwareMap.get(DistanceSensor.class, "C_STOP"),
                telemetry
        );
        // Subsystems
    }

    double twoButtonConvert(boolean positive, boolean negative) {
        return (positive ? 1 : (negative ? -1 : 0));
    }

}
