package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.threads.actions.Action;

import java.util.LinkedList;


//threaded tele op controller......
@Autonomous(name = "LEFT_AutoOp")
@Disabled
public class threadedLeftAutonomousOp extends threadedAutonomousOp {


    LinkedList<Action> auto_actions;
    @Override
    public void init() {
        super.init();


        _move._is_auto=true;

        _liftclaw._is_auto = true;

        auto_actions = new LinkedList<Action>();


        //null action for process to work
        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = false;
            }
        });
        //move forward 1 tile and lift arm
        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = true;
                _move.MoveForward(1.0);
                //_arm_release.setPosition(0.9);
                this._running = false;
            }
        });
        /*
        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = true;
                _move.SlideRight(1.0);
                this._running = false;
            }
        });
        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = true;
                _move.MoveForward(0.5);
                this._running = false;
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = true;
                _move.Rotate90CCW();
                this._running = false;
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = true;
                _move.MoveForward(0.1);
                _liftclaw.runToPos(LiftClawThread.LOW_POS);
                _move.MoveBackward(0.1);
                this._running = false;
            }
        });
*/

        /*
        auto_actions.add(new Action() {
            @Override
            public void run() {
                // move right 1 tile
                this._running = true;
                _move.SlideRight(1.0);
                this._running = false;
            }
        });
        auto_actions.add(new Action() {
            @Override
            public void run() {
                this._running = true;
                // move forward 1.5 tiles
                _move.MoveForward(1.5);
                this._running = false;
            }
        });
*/

        /*
        auto_actions.add(new Action() {
            @Override
            public void run() {
                // rotate CCW 90 degrees
                _move.Rotate90CCW();
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                // move forward 1/4 tile
                _move.MoveForward(0.25);
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                //place cone
                _liftclaw.placeCone();
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                // reverse 1/4 tile
                _move.MoveBackward(0.25);
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                // right 1/2 tile
                _move.SlideRight(0.5);
            }
        });

        auto_actions.add(new Action() {
            @Override
            public void run() {
                //forward 2.5 tiles
                _move.MoveForward(2.5);
            }
        });
        auto_actions.add(new Action() {
            @Override
            public void run() {
                // grab a cone
                _liftclaw.runToPos(LiftClawThread.MEDIUM_POS);
            }
        });



        _autoThread = new Thread() {

            @Override
            public void run() {
                {
                    // now do the things......
                    // lift arm to medium pos
                    new Thread() {
                        @Override
                        public void run() {
                            _liftclaw.runToPos(LiftClawThread.MEDIUM_POS);
                        }
                    }.start();


            /*
                1540 top of 5 stack
                cone stack heights
                1st 0
                2nd 200
                3rd 400
                4th 600
                5th 800
            //* /


                    do {
                        repeat++;



                        //lower arm
                        new Thread() {
                            @Override
                            public void run() {
                                _liftclaw.runToPos(STACK_SET);
                            }
                        }.start();





                        // set heights for next time
                        RETREIVE_HEIGHT = RETREIVE_HEIGHT - RETREIVE_DELTA;
                        //STACK_SET = STACK_SET - RETREIVE_DELTA;

                        // reverse 2.5 tile
                        _move.MoveBackward(2.5);

                        // slide left 1/2 tile
                        _move.SlideLeft(0.5);

                    }
                    while (repeat < 5);
                    // move forward 1/4 tile
                    _move.MoveForward(0.25);

                    //place cone
                    _liftclaw.placeCone();

                    // reverse 1/4 tile
                    _move.MoveBackward(0.25);

                    // slide left 1/2 tile
                    _move.SlideLeft(0.5);

                    // move forward 3-parkLocation
                    _move.MoveForward(3 - _whereToEnd_value);

                    //rotate back toward wall
                    _move.Rotate90CCW();
                    // and done
                }
            }
        };
        _autoThread.start();
        */

        new Thread(auto_actions.getFirst()).start();
    }

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


        if (! auto_actions.isEmpty()) {
            if (! auto_actions.getFirst().isRunning()){
                auto_actions.remove();
                if (! auto_actions.isEmpty()) new Thread(auto_actions.getFirst()).start();
            }
        }
        /*
        if (_whereToEnd_value != 0 ) {
            int cmvid = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            _vuforia = new VuforiaFieldNavigationWebcamThread(
                    webcam,
                    telemetry,
                    cmvid
            );*/
        //_vuforia.start();
        }


    }
