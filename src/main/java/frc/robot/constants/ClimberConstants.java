package frc.robot.constants;

public class ClimberConstants {
    // TODO: update the values in the whole class
    public static final int rightSpoolPort = 5;
    public static final int leftSpoolPort = 6;

    public static final double averageKp = 0.3; // motor percentage/meter
    public static final double averageKf = 0.6; // motor percentage/meter
    public static final double differentialKp = 1.0; // motor percentage/meter
    public static final double differentialRatio = 0.5; // ratio between differential kp outputs

    public static final double climbHeight = 0.22; // meters
    public static final double climbTolerance = 0.1;

    private static final double spoolDiameter = 1 * 2.54 / 100; // meters
    private static final double gearRatio = 1.0 / 12.0;
    public static final double positionFactor = Math.PI * spoolDiameter * gearRatio; // meter/rotation
    public static final double velocityFactor = positionFactor * 60; // (meter/second)/RPM
    public static final double spoolsDistance = 0.48; // meters
}