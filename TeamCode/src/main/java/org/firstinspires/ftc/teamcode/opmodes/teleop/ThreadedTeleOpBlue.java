package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Lights;


//threaded tele op controller......
@TeleOp(name = "TeleOp-Blue")

/**
 * This is the threaded Tele Opmode with blue lights
 * @author Greg Weaver
 */
public class ThreadedTeleOpBlue extends ThreadedTeleOp {

    @Override
    public Lights getLights() {
        return new Lights(hardwareMap.dcMotor.get("LIGHTS")){
            @Override
            public void on() {
                blueOn();
            }
        };
    }
}
