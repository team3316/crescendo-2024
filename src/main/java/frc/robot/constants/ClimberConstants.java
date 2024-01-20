package frc.robot.constants;

public class ClimberConstants {
    // TODO: update the values in the whole class
    public static final int rightSpoolPort = 0;
    public static final int leftSpoolPort = 1;

    public static final double kP = 1; // voltage/meter

    private static final double spoolDiameter = 1; // meters
    private static final double gearRatio = 1.0 / 1.0;
    public static final double positionFactor = Math.PI * spoolDiameter * gearRatio; // meter/rotation
    public static final double velocityFactor = Math.PI * spoolDiameter * gearRatio * 60; // (meter/second)/RPM
    public static final double spoolsDistance = 1; // meters
}