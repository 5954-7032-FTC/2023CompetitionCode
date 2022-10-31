package org.firstinspires.ftc.teamcode.field;

import java.util.Objects;

public class Point {

    private float _X;
    private float _Y;

    public Point(float _X, float _Y) {
        this._X = _X;
        this._Y = _Y;
    }



    public float get_X() {
        return _X;
    }

    public void set_X(float _X) {
        this._X = _X;
    }

    public float get_Y() {
        return _Y;
    }

    public void set_Y(float _Y) {
        this._Y = _Y;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point point = (Point) o;
        return Float.compare(point._X, _X) == 0 && Float.compare(point._Y, _Y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(_X, _Y);
    }
}
