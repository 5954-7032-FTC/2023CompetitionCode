package org.firstinspires.ftc.teamcode.field;

public class Field {

    private static final float mmPerInch        = 25.4f;
    private static final float Field        = 144 * mmPerInch;
    private static final float Tile         = 24 * mmPerInch;
    private static final float halfTile = 12 *mmPerInch;

    private static float GROUND_HEIGHT= (float) (mmPerInch * 0.56);
    private static float LOW_HEIGHT = (float) (mmPerInch * 13.5);
    private static float MEDIUM_HEIGHT = (float) (mmPerInch * 23.5);
    private static float HIGH_HEIGHT = (float) (mmPerInch * 33.5);


    private static Point [] safe_field_locations_for_robot = {
            new Point(halfTile,halfTile)
    };




}
