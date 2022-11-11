package org.firstinspires.ftc.teamcode.util;

public abstract class PID_greg {
    double _kP,_kI,_kD;
    int _error_margin;

    public PID_greg(double kP, double kI, double kD, int error_margin) {
        _kP =kP;
        _kI=kI;
        _kD=kD;
        _error_margin = error_margin;
    }
    public abstract void end(int target);
    public abstract void start(int target);
    public abstract void loop(double power);
    public abstract int getLocation();
    public void action (int target) {
        double integral = 0;
        double derivative;
        double power;
        double error = getLocation()-target;
        double last_error = error;

        start(target);

        long current_tick=System.currentTimeMillis();
        long last_tick = current_tick;
        while (error > _error_margin) {
            current_tick = System.currentTimeMillis();
            error = getLocation()-target;
            integral += (current_tick-last_tick) * error;
            derivative = (current_tick-last_tick)*(error-last_error);
            power = _kP * (error + _kI * integral + _kD * derivative);
            last_tick = current_tick;

            if ( -0.1 < power && power < 0.1) {
                power = 0;
            }
            loop(power);
        }
        end(target);
    }
}