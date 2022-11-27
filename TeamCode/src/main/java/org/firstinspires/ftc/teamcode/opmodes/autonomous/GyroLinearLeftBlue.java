package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.LightBlue;
import org.firstinspires.ftc.teamcode.hardware.Lights;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

@Autonomous(name = "Left-BLUE")
public class GyroLinearLeftBlue extends GyroLinearBase {
    @Override
    public void transitionOnStop() {
        AutoTransitioner.transitionOnStop(this, "TeleOpBlue");
    }

    @Override
    public ColorSensorDevice getColorSensorDevice() {
        return colorSensorDeviceRight;
    }

    @Override
    public void strafeDirection(double distance) {
        driveRight(distance);
    }

    @Override
    public void strafeAntiDirection(double distance) {
        driveLeft(distance);
    }

    @Override
    public void lightOn() {
        light.blueon();
    }
    @Override
    public Lights getLight() {
        return new LightBlue(hardwareMap.dcMotor.get("LIGHTS"));
    }
}