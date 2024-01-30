package frc.robot.util;

import java.util.function.DoubleSupplier;

public class PolarCoordinate {

    /*
     * Converts cartesian coordinates to polar coordinates
     * @param x the x coordinate
     * @param y the y coordinate
     * @return the polar coordinates as a double array with r as the first element and theta as the second
     */
    public static double[] toPolarCoordinate(DoubleSupplier x, DoubleSupplier y){
        return new double[]{
            Math.sqrt((x.getAsDouble() * x.getAsDouble()) + (y.getAsDouble() * y.getAsDouble())),
            Math.atan2(y.getAsDouble(), x.getAsDouble())
        };
    }

    /*
     * Converts polar coordinates to cartesian coordinates
     * @param r the magnitude
     * @param theta the angle in radians
     * @return the cartesian coordinates as a double array with x as the first element and y as the second
     */
    public static double[] toCartesianCoordinate(DoubleSupplier r, DoubleSupplier theta){
        return new double[]{
            r.getAsDouble() * Math.cos(theta.getAsDouble()),
            r.getAsDouble() * Math.sin(theta.getAsDouble())
        };
    }
}
