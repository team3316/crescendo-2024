package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristConstants {
    public static final int wristCANID = 9;
    
    public static final double collectAngle = -110.5; // deg
    public static final double AMPAngle = 0; // deg
    public static final double TRAPAngle = -30.68; // deg
    public static final double underChainAngle = -110.5; // deg

    private static final double gearRatio = 1.0 / 35.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations

    private static final double maxVelocity = 300; // deg/sec
    private static final double maxAcceleration = 300; // deg/sec^2
    public static final TrapezoidProfile.Constraints profileConstrains = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    public static final double kg = 0.15; // v
    public static final double kv = 3 / (Math.PI * 275 / 180); // v/(rad/sec)

    public static final double kp = 0.12; //v/deg

    public static final double softLimitExtraAngle = 10; // deg
}
