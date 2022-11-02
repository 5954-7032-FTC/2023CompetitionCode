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
@Autonomous(name = "RIGHT_AutoOp")
public class threadedRightAutonomousOp extends threadedAutonomousOp {

    @Override
    public void loop() {
        /*
        Things to do:
        1. find where to park for later.
        2. will make a right vs left auto app picker.....makes it easy.

         */

        if (mytime.seconds() >=30 ) this.stop(); // this may be unnecessary

        if ( _whereToEnd_value == 0 && mytime.seconds()>2) {
            setWhereToEnd(null);
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

        // now do the things...... but only once.

        if (doneAuto) return;

        // lift arm to medium pos
        new Thread() {
            @Override
            public void run() {
                _liftclaw.runToPos(LiftClawThread.MEDIUM_POS);
            }
        }.start();

        // move right 1 tile
        _move.SlideRight(1.0);

        // move forward 1.5 tiles
        _move.MoveForward(1.5);

        // rotate CCW 90 degrees
        _move.Rotate90CCW();

            /*
                1540 top of 5 stack
                cone stack heights
                1st 0
                2nd 200
                3rd 400
                4th 600
                5th 800
            */

        final int STACK_SET = 1540;
        int RETREIVE_HEIGHT = 800;
        int RETREIVE_DELTA = 200;
        int repeat = 0;
        do {
            repeat++;
            // move forward 1/4 tile
            _move.MoveForward(0.25);

            //place cone
            _liftclaw.placeCone();

            // reverse 1/4 tile
            _move.MoveBackward(0.25);

            // right 1/2 tile
            _move.SlideLeft(0.5);

            //lower arm
            new Thread(){
                @Override
                public void run() {
                    _liftclaw.runToPos(STACK_SET);
                }
            }.start();

            //forward 2.5 tiles
            _move.MoveForward(2.5);

            // grab a cone
            _liftclaw.runToPos(RETREIVE_HEIGHT);

            // set heights for next time
            RETREIVE_HEIGHT = RETREIVE_HEIGHT - RETREIVE_DELTA;
            //STACK_SET = STACK_SET - RETREIVE_DELTA;

            // reverse 2.5 tile
            _move.MoveBackward(2.5);

            // slide left 1/2 tile
            _move.SlideRight(0.5);

        }
        while (repeat < 5);
        // move forward 1/4 tile
        _move.MoveForward(0.25);

        //place cone
        _liftclaw.placeCone();

        // reverse 1/4 tile
        _move.MoveBackward(0.25);

        // slide left 1/2 tile
        _move.SlideRight(0.5);

        // move forward 3-parkLocation
        _move.MoveForward( 3 - _whereToEnd_value );

        //rotate back toward wall
        _move.Rotate90CW();
        // and done

        this.terminateOpModeNow();
    }

 }
