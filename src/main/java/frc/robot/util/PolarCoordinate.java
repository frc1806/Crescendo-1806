package frc.robot.util;

import java.util.function.DoubleSupplier;

public class PolarCoordinate {

    public static double[] toPolarCoordinate(DoubleSupplier x, DoubleSupplier y){
        return new double[]{
            Math.sqrt((x.getAsDouble() * x.getAsDouble()) + (y.getAsDouble() * y.getAsDouble())),
            Math.atan2(y.getAsDouble(), x.getAsDouble())
        };
    }
}
