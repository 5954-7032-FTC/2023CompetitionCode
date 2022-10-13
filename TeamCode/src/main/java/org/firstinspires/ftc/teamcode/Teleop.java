package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.algorithms.*;
 
@TeleOp(name = "TeleOp")

public class Teleop extends LinearOpMode {

    final static MechanumDrive drive = new MechanumDrive();
    //final static LeftLift lLift = new LeftLift();
    //final static RightLift RLift = new RightLift();
    final static motorRampProfile Joy1Y = new motorRampProfile(1.5), Joy1X = new motorRampProfile(1.5), Joy2X = new motorRampProfile(1.5);
    //final static ChasisSensors CSensors = new ChasisSensors();
    int p = 0;



    int toNum(boolean in) {
        return (in?1:0);
    }

    //float degreesFromStick()
    @Override
    public void runOpMode() throws InterruptedException {

        this.Initialize();
        telemetry.addData("Robot Ready", "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            //mechanum.calcMove(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_y);
            // mechanum.M[0-3] has how much each motor should move
            // not sure how to turn this into a useful bit for the PID controlller in this mode....


            drive.Drive(Joy1Y.ramp(gamepad1.left_stick_y,1.5), Joy1X.ramp(gamepad1.left_stick_x, 1.5),Joy2X.ramp(gamepad1.right_stick_x, 1.5));


            /*
            RLift.MoveDropper(0.7 * twoButtonConvert(gamepad2.right_bumper,gamepad2.right_trigger > 0.2));
            RLift.MoveDucker(0.6 * twoButtonConvert(gamepad2.x,gamepad2.b));
            if (RLift.ReadArmLimit()) RLift.ZeroPotentiometer();

            lLift.MoveElevator(twoButtonConvert(gamepad2.dpad_up, gamepad2.dpad_down));
            lLift.MoveSlide(twoButtonConvert(gamepad2.dpad_left, gamepad2.dpad_right));
            lLift.MoveIntake(twoButtonConvert(gamepad2.left_bumper, gamepad2.left_trigger > 0.2));

            // handle RLift motion
            if(RLift.shoulderPID.isActive()){
                RLift.MoveElevator(RLift.shoulderPID.getOutput());
            }
            if (Math.abs(gamepad2.left_stick_y) - 0.3 >  0 || !RLift.shoulderPID.isActive()){
                if(RLift.shoulderPID.isActive())RLift.shoulderPID.Deactivate();
                RLift.MoveElevator(gamepad2.left_stick_y);
            }

            if(RLift.elbowPID.isActive()){
                RLift.MoveElbow(RLift.elbowPID.getOutput());
            }
            if (Math.abs(gamepad2.right_stick_y) - 0.3 >  0 || !RLift.elbowPID.isActive()){
                if(RLift.elbowPID.isActive()) RLift.elbowPID.Deactivate();
                RLift.MoveElbow(gamepad2.right_stick_y);
            }


            if (gamepad1.a){
                p = 1;
            } else if (gamepad1.b){
                p = 2;
            } else if (gamepad2.a){
                p = 0;
            }

            if (gamepad1.dpad_down){
                p = 2;
            } else if (gamepad1.dpad_right){
                p = 1;
            } else if (gamepad1.dpad_up){
                p = 0;
            }

            if(gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down){
                if(!RLift.elbowPID.isActive())RLift.elbowPID.Activate();
                if(!RLift.shoulderPID.isActive())RLift.shoulderPID.Activate();
                RLift.elbowPID.setSetPoint(RLift.GOAL_ARM_LEVELS[p].getElbowEnc());
                RLift.shoulderPID.setSetPoint(RLift.GOAL_ARM_LEVELS[p].getArmPot());
                telemetry.addData("Arm Level", RLift.GOAL_ARM_LEVELS[p].getName());
            }

            if(gamepad1.a || gamepad1.b || gamepad2.a){
                if(!RLift.elbowPID.isActive())RLift.elbowPID.Activate();
                if(!RLift.shoulderPID.isActive())RLift.shoulderPID.Activate();
                RLift.elbowPID.setSetPoint(RLift.POSITIONS[p].getElbowEnc());
                RLift.shoulderPID.setSetPoint(RLift.POSITIONS[p].getArmPot());
                telemetry.addData("Arm Level", RLift.POSITIONS[p].getName());
            }

            telemetry.addLine("Right Lift Sensors: ")
                    .addData("Arm Angle: ", RLift.ReadPotentiometer())
                    .addData("Elbow Encoder: ", RLift.ReadElbowEncoder())
                    .addData("Elbow PID Out: ", RLift.elbowPID.getOutput())
                    .addData("Elbow PID Act", RLift.elbowPID.isActive());

            telemetry.update();
            */
            idle();
        }
        //RLift.elbowPID.Deactivate();
        //RLift.shoulderPID.Deactivate();
    }


    public void Initialize(){
        final DcMotor init_drive[] = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};

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

    double twoButtonConvert(boolean positive, boolean negative){
        return (positive ? 1 : (negative ? -1 : 0));
    }

}
