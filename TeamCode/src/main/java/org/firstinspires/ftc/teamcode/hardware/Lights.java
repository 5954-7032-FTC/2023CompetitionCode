package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lights {

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
    public void off() {
        lights.setPower(0);
    }
}
