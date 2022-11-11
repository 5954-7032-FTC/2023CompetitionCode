package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.threads.LiftClawThread;
import org.firstinspires.ftc.teamcode.threads.MovementThread;


//threaded tele op controller......
@TeleOp(name = "TeleOpOld")
@Disabled
public class OldThreadedTeleOp extends OpMode {

    MovementThread _move;
    LiftClawThread _liftclaw;

    Telemetry.Item _threadCount;
    Servo _arm_release;

    @Override
    public void init() {
        telemetry.setAutoClear(false);
        // set up MovementThread
        final DcMotor [] motors = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};

        _move = new MovementThread(gamepad1, motors,telemetry);


        // setup LiftClawOld
        final DcMotor lift_motor = hardwareMap.dcMotor.get("LIFT");
        final Servo [] lift_servos = {
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };
        final TouchSensor bottom_stop = hardwareMap.touchSensor.get("BSTOP");


        final DistanceSensor post_sensor = hardwareMap.get(DistanceSensor.class, "C_STOP");

        _liftclaw = new LiftClawThread(
                lift_motor,
                lift_servos,
                bottom_stop,
                post_sensor,
                telemetry,
                gamepad2
                );

        _threadCount = telemetry.addData("Threads", Thread.activeCount());

        _arm_release =  hardwareMap.servo.get("ARM_RELEASE");
    }

    @Override
    public void start() {
        _move.start();
        _liftclaw.start();
    }


    @Override
    public void loop() {
        _threadCount.setValue(Thread.activeCount());
        telemetry.update();
        if (gamepad1.left_bumper) {
            _arm_release.setPosition(0.4);
        }
        if (gamepad1.right_bumper) {
            _arm_release.setPosition(0.7);
        }
    }

    @Override
    public void stop() {
        super.stop();
        _move.cancel();
        _liftclaw.cancel();
    }


}
