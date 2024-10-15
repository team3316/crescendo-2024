package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristConstants {
    public static final int wristCANID = 9;
    public static final int wristHallEffectID = 2;
    
    public static final double collectAngle = -109.8; // deg
    public static final double AMPAngle = 0; // deg
    public static final double preClimbAngle = -109.3; // deg

    // The angle in which the wrist entes the range of the hall effect
    public static final double hallEffectAngle = -99.4; // deg

    private static final double gearRatio = 1.0 / 35.0; 
    
    public static final double positionFactor = 360 * gearRatio; // deg/rotations

    private static final double maxVelocity = 600; // deg/sec
    private static final double maxAcceleration = 1200; // deg/sec^2, measured max acceleration - 1400
    public static final TrapezoidProfile.Constraints profileConstrains = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    public static final double kg = 0.15; // v
    public static final double kv = 3 / (Math.PI * 275 / 180); // v/(rad/sec)

    public static final double kp = 0.12; //v/deg

    public static final double softLimitExtraAngle = 10; // deg
}
