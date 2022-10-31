package org.firstinspires.ftc.teamcode.field;

public class Junction extends Rectangle {

    Point _position;
    float _height;
    public Junction(Point topLeft, Point bottomRight, Point position, float height) {
        super(topLeft, bottomRight);
        _position = position;
        _height = height;
    }
}
