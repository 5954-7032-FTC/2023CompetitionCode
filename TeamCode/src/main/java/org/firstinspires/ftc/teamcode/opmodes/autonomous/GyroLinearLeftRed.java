package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

@Autonomous(name = "Left-RED")
public class GyroLinearLeftRed extends GyroLinearBase {
    @Override
    public void transitionOnStop() {
        AutoTransitioner.transitionOnStop(this, "TeleOpRed");
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
        light.redon();
    }

    @Override
    public Lights getLight() {
        return new LightRed(hardwareMap.dcMotor.get("LIGHTS"));
    }
}