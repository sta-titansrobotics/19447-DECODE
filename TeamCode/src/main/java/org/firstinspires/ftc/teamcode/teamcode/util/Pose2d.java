package com.teamcode.util;

public class Pose2d {
    public double x;      // inches
    public double y;      // inches (left = +)
    public double heading; // radians, CCW+, field frame

    public Pose2d() { this(0,0,0); }

    public Pose2d(double x, double y, double heading) {
        this.x = x; this.y = y; this.heading = Angle.norm(heading);
    }

    public Pose2d copy() { return new Pose2d(x, y, heading); }
}
