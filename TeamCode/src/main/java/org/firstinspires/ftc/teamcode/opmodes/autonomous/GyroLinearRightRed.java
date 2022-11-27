package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.LightRed;
import org.firstinspires.ftc.teamcode.hardware.Lights;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

@Autonomous(name = "Right-RED")
public class GyroLinearRightRed extends GyroLinearBase {
    @Override
    public void transitionOnStop() {
        AutoTransitioner.transitionOnStop(this, "TeleOpRed");
    }

    @Override
    public ColorSensorDevice getColorSensorDevice() {
        return colorSensorDeviceLeft;
    }

    @Override
    public void strafeDirection(double distance) {
        driveLeft(distance);
    }

    @Override
    public void strafeAntiDirection(double distance) {
        driveRight(distance);
    }

    @Override
    public void lightOn() {
        light.redon();
    }
    @Override
    public Lights getLight() {
        return new LightRed(hardwareMap.dcMotor.get("LIGHTS"));
    }
}