package org.firstinspires.ftc.teamcode.threads;

public class RobotThread extends Thread {
    private boolean _isCancelled  = false;
    public void cancel() {
        _isCancelled = true;
    }

    public boolean isCancelled() {
        return _isCancelled;
    }

}
