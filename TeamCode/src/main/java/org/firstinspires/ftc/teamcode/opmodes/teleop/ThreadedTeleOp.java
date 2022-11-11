package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ArmRelease;
import org.firstinspires.ftc.teamcode.threads.LiftClawThread;
import org.firstinspires.ftc.teamcode.threads.TweakableMovementThread;


//threaded tele op controller......
@TeleOp(name = "TeleOp")
public class ThreadedTeleOp extends OpMode {

    TweakableMovementThread _move;
    LiftClawThread _liftclaw;
    ArmRelease _armRelease;

    Telemetry.Item _threadCount;
    private BNO055IMU imu         = null;

    @Override
    public void init() {
        telemetry.setAutoClear(false);
        // set up MovementThread
        final DcMotor [] motors = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        _move = new TweakableMovementThread(gamepad1, motors,telemetry,imu,500);


        // setup LiftClaw
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

        _armRelease =  new ArmRelease(hardwareMap.servo.get("ARM_RELEASE"));
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
            _armRelease.set();
        }
        if (gamepad1.right_bumper) {
            _armRelease.release();
        }
    }

    @Override
    public void stop() {
        super.stop();
        _move.cancel();
        _liftclaw.cancel();
    }


}
