package org.firstinspires.ftc.teamcode.threads.actions;

public abstract class  Action  implements Runnable {
    protected boolean _running = false;
    abstract public void run();
    public boolean isRunning() {
        return _running;
    }
}