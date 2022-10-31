package org.firstinspires.ftc.teamcode.field;

public class Robot extends Rectangle {
    private Point _position;
    private float _heading;

    public Robot(Rectangle bounds, Point position, float heading) {
        super(bounds.topLeft,bounds.bottomRight);
        this._position = position;
        this._heading = heading;
    }

    public Point getPosition() {
        return _position;
    }

    public void setPosition(Point _position) {
        this._position = _position;
    }

    public float getHeading() {
        return _heading;
    }

    public void setHeading(float heading) {
        this._heading = heading;
    }

}
