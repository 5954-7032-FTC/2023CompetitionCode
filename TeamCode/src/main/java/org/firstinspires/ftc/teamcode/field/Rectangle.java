package org.firstinspires.ftc.teamcode.field;

public class Rectangle {
    Point topLeft;
    Point bottomRight;


    public Rectangle(Point topLeft, Point bottomRight) {
        this.topLeft = topLeft;
        this.bottomRight = bottomRight;
    }

    public Point getTopLeft() {
        return topLeft;
    }

    public void setTopLeft(Point topLeft) {
        this.topLeft = topLeft;
    }

    public Point getBottomRight() {
        return bottomRight;
    }

    public void setBottomRight(Point bottomRight) {
        this.bottomRight = bottomRight;
    }


    public boolean checkCollision(Rectangle rect) {
        return !(
                this.bottomRight.get_X() < rect.topLeft.get_X()
                || this.topLeft.get_X() > rect.bottomRight.get_X()
                || this.topLeft.get_Y() < rect.bottomRight.get_Y()
                || this.bottomRight.get_Y() > rect.topLeft.get_Y()
                );

    }
}
