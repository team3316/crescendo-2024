package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristConstants {
    public static final int wristCANID = 0;
    
    public static final double collectAngle = 0; // deg
    public static final double AMPAngle = 0; // deg
    public static final double TRAPAngle = 0; // deg
    public static final double underChainAngle = 0; // deg

    private static final double maxVelocity = 0; // deg/sec
    private static final double maxAcceleration = 0; // deg/sec^2
    public static final TrapezoidProfile.Constraints profileConstrains = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    public static final double kg = 0; // v
    public static final double kv = 0; // v/(rad/sec)

    public static final double kp = 0; //v/deg
    
    private static final double gearRatio = 1.0 / 64.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations

    public static final double softLimitExtraAngle = 10; // deg
}
