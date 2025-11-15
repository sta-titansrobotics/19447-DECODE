package com.teamcode.util;

public final class Angle {
    private Angle() {}

    /**
     * Normalize angle to [-π, π).
     * Handles extreme angles efficiently.
     * Note: Returns values in [-π, π) (inclusive lower, exclusive upper).
     */
    public static double norm(double a) {
        // Fast path for already-normalized angles
        if (a >= -Math.PI && a < Math.PI) return a;

        // Handle extreme angles with modulo
        a = a % (2 * Math.PI);
        if (a < -Math.PI) a += 2*Math.PI;
        if (a >= Math.PI)  a -= 2*Math.PI;
        return a;
    }

    public static double shortestDiff(double to, double from) {
        return norm(to - from);
    }

    public static double lerpAngle(double from, double to, double t) {
        double d = shortestDiff(to, from);
        return norm(from + d * t);
    }
}
