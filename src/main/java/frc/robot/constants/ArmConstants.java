package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int leaderCANID = 8; // left
    public static final int followerCANID = 7; // right
    public static final int leftSwitchPort = 6;
    public static final int rightSwitchPort = 7;

    public static final double collectAngle = -43.2; // deg
    public static final double TRAPAngle = 105; // deg
    public static final double underChainAngle = 0; // deg
    public static final double AMPAngle = 56; // deg
    public static final double ALIGNAngle = 36.86; // deg
    
    private static final double gearRatio = 1.0 / 192.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations
    public static final double velocityFactor = 360 * gearRatio / 60; // (deg/sec)/RPM

    private static final double maxVelocity = 150; // deg/sec
    private static final double maxAcceleration = 150; // deg/sec^2
    public static final TrapezoidProfile.Constraints profileConstrains = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    public static final double ks = 0; // v
    public static final double kg = 0.233; // v
    public static final double kv = 3.72; // v/(rad/sec)
    public static final double ka = 0; // v/(rad/sec^2)

    public static final double kp = 0.01; // percentage/deg

    public static final float softLimitExtraAngle = 10; // deg
}
