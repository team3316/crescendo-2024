package frc.robot.constants;

public class ClimberConstants {
    // TODO: update the values in the whole class
    public static final int rightSpoolPort = 5;
    public static final int leftSpoolPort = 6;

    public static final double averageKp = 1; // voltage/meter
    public static final double averageKf = 1; // voltage/meter
    public static final double differentialKp = 1; // motor percentage/meter

    public static final double climbHeight = 1; // meters

    private static final double spoolDiameter = 1; // meters
    private static final double gearRatio = 1.0 / 1.0;
    public static final double positionFactor = Math.PI * spoolDiameter * gearRatio; // meter/rotation
    public static final double velocityFactor = positionFactor * 60; // (meter/second)/RPM
    public static final double spoolsDistance = 1; // meters
}