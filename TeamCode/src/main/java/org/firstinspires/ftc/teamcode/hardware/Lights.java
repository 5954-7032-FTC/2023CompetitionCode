package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Lights {

    DcMotor lights;

    public Lights(DcMotor lights) {
        this.lights = lights;
    }

    public void redon() {
        lights.setPower(1);
    }
    public void blueon() {
        lights.setPower(-1);
    }

    private long blink_min=1000; //500 milliseconds

    private long last_blink_time=-1;
    public void blink() {
        if (System.currentTimeMillis() > last_blink_time + blink_min) {
            off();
            on();
            last_blink_time = System.currentTimeMillis();
        }

    }
    public void off() {
        lights.setPower(0);
    }
    public abstract void on();
}
