package org.firstinspires.ftc.teamcode.threads;

import java.util.ArrayList;


public class AutoThread extends RobotThread {
    boolean _isLeft = true;
    public AutoThread(boolean isLeft) {
        _isLeft = isLeft;
    }

    class Tasks {
    }




    public static ArrayList<Tasks> taskList;

    public void run() {
        taskList = new ArrayList<Tasks>(20);

        taskList.add( new Tasks());

        while (!isCancelled()) {
            //do tasks in order

            /*
            Let's list the tasks....
            1. find the parking spot
            2. raise the arm
            3. move to medium post
            4. place first cone
            repeat next 4 steps 5 times:
                5. move to stack
                6. pull first from stack
                7. move to post
                8 place on post
            9. move to parking spot
            */
        }
    }

}
