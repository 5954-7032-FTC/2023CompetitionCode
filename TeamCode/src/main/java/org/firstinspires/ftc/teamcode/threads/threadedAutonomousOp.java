package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Point;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.charset.StandardCharsets;


//threaded tele op controller......
@Autonomous(name = "AutonomousOp")
public class threadedAutonomousOp extends OpMode {

    MovementThread _move;
    LiftClawThread _liftclaw;
    TensorFlowThread _tensorFlow;
    VuforiaFieldNavigationWebcamThread _vuforia;

    AutoThread _myauto;
    Telemetry.Item _threadCount;
    Telemetry.Item _whereToEnd;

    Servo _arm_release;

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
                gpe
                );

        _threadCount = telemetry.addData("Threads", Thread.activeCount());


        WebcamName camera =  hardwareMap.get(WebcamName.class, "Webcam 1");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        _tensorFlow = new TensorFlowThread( tfodMonitorViewId, telemetry,camera,this);


        /*
        int cmvid = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        _vuforia = new VuforiaFieldNavigationWebcamThread(
                webcam,
                telemetry,
                cmvid
        );
*/
        _whereToEnd.addData("Where?","nope");
        AutoTransitioner.transitionOnStop(this, "TeleOp");
        mytime = new ElapsedTime();

        _myauto = new AutoThread(true, _move, _liftclaw,_vuforia);

    }


    ElapsedTime mytime;
    @Override
    public void start() {
        mytime.reset();
        _arm_release =  hardwareMap.servo.get("ARM_RELEASE");
        new Thread() {
            @Override
            public void run() {
                super.run();
                _arm_release.setPosition(0);
                try {
                    Thread.sleep(6000);
                }
                catch (Exception ignored) {
                }
                _arm_release.setPosition(.5);
            }
        }.start();

        _move.start();
        _liftclaw.start();
        _tensorFlow.start();

        _liftclaw.Calibrate();
        _myauto.start();

    }


    @Override
    public void loop() {
        /*
        Things to do:
        1. find where to park for later.
        2. find my location
        3. determine proper color and pattern
            blue:
                left:

            blue:
                right:
                    1. turn right 90 degrees, move forward 1 unit drop cone
                    2. raise claw, turn left 90 degrees, move forward 2 units, turn right, move forward 1/2 unit, drop lift, raise lift, reverse 1 unit, turn left 90 degrees, move forward 1/2 unit, lower lift to ground, release cone, lift claw.
                    3. raise claw, reverse 1/2 unit, turn right 90, forward 1/2, lower lift to cone, lift claw, reverse 1/2

         */

        if (mytime.seconds() >=30 ) this.stop(); // this may be unnecessary

        if ( _whereToEnd_value == 0 && mytime.seconds()>2) {
            setWhereToEnd(null);
        }

        if (mytime.seconds() >=24) {
            // move to final location X, Y to be determined
            Point [] final_spots = {
                    new Point(0,0),
                    new Point(1,0);
                    new Point(2,0);
            }
            _myauto.cancel();
        }

        if (_whereToEnd_value != 0 ) {
            int cmvid = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            _vuforia = new VuforiaFieldNavigationWebcamThread(
                    webcam,
                    telemetry,
                    cmvid
            );
            _vuforia.start();

        }

        // do the autonomous things


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
    }
 }
