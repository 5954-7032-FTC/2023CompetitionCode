package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;


//threaded tele op controller......
@TeleOp(name = "TeleOpVision")
public class threadedTeleOpVision extends OpMode {

    MovementThread _move;
    LiftClawThread _liftclaw;
    VuforiaFieldNavigationWebcamThread _vuforia;
    TensorFlowThread _tensor;

    Telemetry.Item _threadCount;

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

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        _vuforia = new VuforiaFieldNavigationWebcamThread(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                telemetry,
                cameraMonitorViewId
        );
        */

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




        WebcamName camera =  hardwareMap.get(WebcamName.class, "Webcam 1");
        int tfodmonitorid = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        _tensor = new TensorFlowThread(tfodmonitorid,telemetry,camera);

    }

    @Override
    public void start() {
        _move.start();
        _liftclaw.start();
        _tensor.start();
        //_vuforia.start();
    }


    @Override
    public void loop() {





        _threadCount.setValue(Thread.activeCount());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        _move.cancel();
        _liftclaw.cancel();
    }


}
