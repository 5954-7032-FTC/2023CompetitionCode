package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

            //mechanum.calcMove(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_y);
            // mechanum.M[0-3] has how much each motor should move
            // not sure how to turn this into a useful bit for the PID controlller in this mode....


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





            /*
            telemetry.addLine("Right Lift Sensors: ")
                    .addData("Arm Angle: ", RLift.ReadPotentiometer())
                    .addData("Elbow Encoder: ", RLift.ReadElbowEncoder())
                    .addData("Elbow PID Out: ", RLift.elbowPID.getOutput())
                    .addData("Elbow PID Act", RLift.elbowPID.isActive());

            telemetry.update();
            */
            idle();
        }

    }


    public void Initialize() {
        final DcMotor init_drive[] = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};

        final DcMotor init_lift = hardwareMap.dcMotor.get("LIFT");
        final Servo servos[] = {
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };


        lift.Initialize(init_lift,
                servos,
                hardwareMap.touchSensor.get("BSTOP"),
                hardwareMap.opticalDistanceSensor.get("C_STOP"),
                telemetry
        );
        // Subsystems
        drive.Initialize(init_drive);
        /*lLift.Initialize(hardwareMap.dcMotor.get("M_LA"),
                hardwareMap.dcMotor.get("M_SL"),
                hardwareMap.dcMotor.get("M_IL"),
                hardwareMap.get(Rev2mDistanceSensor.class, "I_CD"));
        RLift.Initialize(hardwareMap.dcMotor.get("M_RA"),
                hardwareMap.dcMotor.get("M_RE"),
                hardwareMap.get(CRServo.class,"CR_C"),
                hardwareMap.get(CRServo.class,"CR_D"),
                hardwareMap.analogInput.get("A_A"),
                hardwareMap.get(RevTouchSensor.class,"D_CL"),
                hardwareMap.get(RevTouchSensor.class,"D_AL"),
                hardwareMap.get(RevTouchSensor.class, "D_EM"));
        CSensors.Initialize(hardwareMap.get(RevColorSensorV3.class, "I_CL"),
                hardwareMap.get(RevColorSensorV3.class, "I_CR"));

        RLift.ZeroPotentiometer();

         */
    }

    double twoButtonConvert(boolean positive, boolean negative) {
        return (positive ? 1 : (negative ? -1 : 0));
    }

}
