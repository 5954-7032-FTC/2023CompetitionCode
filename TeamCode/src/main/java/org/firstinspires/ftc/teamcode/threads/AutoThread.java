package org.firstinspires.ftc.teamcode.threads;

import java.util.ArrayList;


public class AutoThread extends RobotThread {
    boolean _isLeft;
    MovementThread _move;
    LiftClawThread _liftClaw;
    VuforiaFieldNavigationWebcamThread _vuforia;
    public AutoThread(boolean isLeft, MovementThread move, LiftClawThread liftClaw, VuforiaFieldNavigationWebcamThread vuforia) {
        _isLeft = isLeft;
        _move = move;
        _liftClaw = liftClaw;
        _vuforia = vuforia;
    }

    interface Task {
        void finishTask();
    }

    class MoveTask extends Thread implements Task {
        FieldPosition _pos;
        public MoveTask(FieldPosition pos) {
            this._pos = pos;
        }
        public void run() {
            _move.DriveTo(_pos);
            finishTask();
        }
        @Override
        public void finishTask() {
        }
    }

    class LiftTask extends Thread implements  Task {
        int _pos;
        public LiftTask(int pos) {
            this._pos = pos;
        }
        public void run() {
            _liftClaw.runToPos(_pos);
            finishTask();
        }
        @Override
        public void finishTask() {
        }
    }

    public static ArrayList<Task> taskList;

    public void run() {
        taskList = new ArrayList<Task>(20);

        taskList.add( new Task() {
            public void finishTask() {
                setTaskDone();
            }

            @Override
            public void finishTask() {

            }
        });

        while (!isCancelled()) {
            //do tasks in order

            /*
            Let's list the tasks....
            1. find the parking spot
            2. rotate 90 (ccw if left, cw if right)
            2. raise the arm
            3. move to medium post
            4. place first cone
            repeat next 4 steps 5 times:
                5. move to stack
                6. pull first from stack
                7. move to post
                8 place on post
            9. move to parking spot
            10. exit
            */
        }
    }

}
