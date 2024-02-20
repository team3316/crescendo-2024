package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int leaderCANID = 8; // left
    public static final int followerCANID = 7; // right
    public static final int leftSwitchPort = 2;
    public static final int rightSwitchPort = 4;

    public static final double collectAngle = -43.2; // deg
    public static final double TRAPAngle = 96.42; // deg
    public static final double underChainAngle = 0; // deg
    public static final double AMPAngle = 49.7; // deg
    public static final double ALIGNAngle = 36.86; // deg
    
    private static final double gearRatio = 1.0 / 64.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations
    public static final double velocityFactor = 360 * gearRatio / 60; // (deg/sec)/RPM

    private static final double maxVelocity = 300; // deg/sec
    private static final double maxAcceleration = 300; // deg/sec^2
    public static final TrapezoidProfile.Constraints profileConstrains = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    public static final double ks = 0; // v
    public static final double kg = 0.7; // v
    public static final double kv = 1.24; // v/(rad/sec) 3 / (Math.PI * 125 / 180)
    public static final double ka = 0; // v/(rad/sec^2)

    public static final double kp = 0.01; // percentage/deg

    public static final float softLimitExtraAngle = 10; // deg
}
