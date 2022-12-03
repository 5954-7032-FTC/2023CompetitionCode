package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LightBlue;
import org.firstinspires.ftc.teamcode.hardware.Lights;


//threaded tele op controller......
@TeleOp(name = "TeleOp-Blue")
public class ThreadedTeleOpBlue extends ThreadedTeleOpBase{
    @Override
    public Lights getLights() {
        return new LightBlue(hardwareMap.dcMotor.get("LIGHTS"));
    }
}
