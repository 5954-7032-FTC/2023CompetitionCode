package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Hashtable;

public class TelemetryDebug {
    private static Hashtable<String, Telemetry.Item> items;

    public TelemetryDebug() {
        items = new Hashtable<>();
    }

}
