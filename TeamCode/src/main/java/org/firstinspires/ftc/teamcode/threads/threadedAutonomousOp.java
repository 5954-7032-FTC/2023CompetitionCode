package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


//threaded tele op controller......

 abstract public class threadedAutonomousOp extends OpMode {


    MovementThread _move;
    LiftClawThread _liftclaw;
    TensorFlowThread _tensorFlow;
    VuforiaFieldNavigationWebcamThread _vuforia;

    //AutoThread _myauto;
    Telemetry.Item _threadCount;
    Telemetry.Item _whereToEnd;

    Servo _arm_release;

    boolean doneAuto=false;
    @Override
    public void init() {
        telemetry.setAutoClear(false);

        GamepadEmpty gpe = new GamepadEmpty();

        // set up MovementThread
        final DcMotor [] motors = {
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")};

        _move = new MovementThread(gpe, motors,telemetry);


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
                gpe,
                _move
                );

        _threadCount = telemetry.addData("Threads", Thread.activeCount());


        WebcamName camera =  hardwareMap.get(WebcamName.class, "Webcam 1");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      //  _tensorFlow = new TensorFlowThread( tfodMonitorViewId, telemetry,camera,this);

      //  _tensorFlow.start();
/*
        int cmvid = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        _vuforia = new VuforiaFieldNavigationWebcamThread(
                webcam,
                telemetry,
                cmvid
        );
        */

        _whereToEnd = telemetry.addData("Where?","nope");
        AutoTransitioner.transitionOnStop(this, "TeleOp");
        mytime = new ElapsedTime();


    }


    ElapsedTime mytime;
    @Override
    public void start() {
        mytime.reset();
        _arm_release =  hardwareMap.servo.get("ARM_RELEASE");

        _liftclaw.Calibrate();

    //    _move.start();
    //    _liftclaw.start();
//        _tensorFlow.start();




        //this.onStart();

    }


    @Override
    public void stop() {
        super.stop();
        _move.cancel();
        _liftclaw.cancel();
    }

    int _whereToEnd_value=0;
    public void setWhereToEnd(String  whereToEnd) {
        if (whereToEnd == null) {
            _whereToEnd_value = (int)Math.floor(Math.random()*100%3)+1;
        }
        else {
            _whereToEnd_value = whereToEnd.getBytes()[0] - '0';
        }
        _whereToEnd.setValue(_whereToEnd_value);
        _tensorFlow.cancel();
        /*
        int cmvid = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        _vuforia = new VuforiaFieldNavigationWebcamThread(
                webcam,
                telemetry,
                cmvid
        );
         */
    }
 }
