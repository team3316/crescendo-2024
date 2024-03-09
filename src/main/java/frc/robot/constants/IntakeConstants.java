package frc.robot.constants;

import frc.robot.motors.PIDFGains;

public class IntakeConstants {
    public final static int intakeMotorID = 11;
    public static final int rollerID = 6;
    public final static int sensorID = 5;

    public final static double collectingPercentage = 1;
    public final static double ejectPercentage = -0.8;
    public final static double disabledPrecent = 0;

    public final static double intakeKp = 0;
    public final static double rollerkp = 0;

    public final static PIDFGains intakeGains = new PIDFGains(intakeKp, 0, 0, 0);
    public final static PIDFGains rollerGains = new PIDFGains(rollerkp, 0, 0, 0);

    private static final double gearRatio = 1.0 / 9.0;
    private static final double rollerCircumference = Math.PI * 0.0508; // in meters
    private static final double intakeCircumference = Math.PI * 0.0254; // in meters
    public static final double intakePositionFactor = intakeCircumference * gearRatio; // m/rotations
    public static final double intakeVelocityFactor = intakeCircumference * gearRatio / 60; // (m/sec)/RPM

    public static final double rollerPositionFactor = rollerCircumference * gearRatio; // m/rotations
    public static final double rollerVelocityFactor = rollerCircumference * gearRatio / 60; // (m/sec)/RPM

}
