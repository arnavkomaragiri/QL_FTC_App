package org.firstinspires.ftc.teamcode.control;

public class QL_Vector2d {

    public static final double EPSILON = 0.00001;

    private final double x, y;

    public QL_Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public QL_Vector2d copy() {
        return new QL_Vector2d(x, y);
    }

    public QL_Vector2d normalized() {
        double norm = norm();
        if (norm < EPSILON) {
            return new QL_Vector2d(1, 0);
        } else {
            return multiplied(1.0 / norm());
        }
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public QL_Vector2d normalize(QL_Vector2d other){
        double x = (this.x + other.x) / 2.0;
        double y = (this.y + other.y) / 2.0;

        return new QL_Vector2d(x, y);
    }

    public double dot(QL_Vector2d other) {
        return x * other.x() + y * other.y();
    }

    public QL_Vector2d multiplied(double scalar) {
        return new QL_Vector2d(scalar * x, scalar * y);
    }

    public QL_Vector2d added(QL_Vector2d other) {
        return new QL_Vector2d(x + other.x, y + other.y);
    }

    public QL_Vector2d negated() {
        return new QL_Vector2d(-x, -y);
    }

    public QL_Vector2d rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new QL_Vector2d(newX, newY);
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof QL_Vector2d) {
            QL_Vector2d otherVector = (QL_Vector2d) other;
            return Math.abs(x - otherVector.x) < EPSILON && Math.abs(y - otherVector.y) < EPSILON;
        }
        return false;
    }

    @Override
    public String toString() {
        return "<" + x + ", " + y + ">";
    }

    public static double getCosAngle(QL_Vector2d v1, QL_Vector2d v2) {
        double dot = v1.x * v2.x + v1.y * v2.y;
        return dot / (v1.norm() * v2.norm());
    }

    public static double distance(QL_Vector2d v1, QL_Vector2d v2) {
        return v1.added(v2.negated()).norm();
    }
}
