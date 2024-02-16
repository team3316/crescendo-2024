package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int leaderCANID = 0;
    public static final int followerCANID = 0;
    public static final int jointCANID = 0;

    public static final double collectAngle = 0; // deg
    public static final double TRAPAngle = 0; // deg
    public static final double underChainAngle = 0; // deg
    public static final double AMPAngle = 0; // deg
    public static final double collectManipulatorAngle = 0;
    public static final double AMPManipulatorAngle = 0;
    public static final double TRAPManipulatorAngle = 0;
    public static final double underChainManipulatorAngle = 0;

    private static final double maxArmVelocity = 0; // deg/sec
    private static final double maxArmAcceleration = 0; // deg/sec^2
    public static final TrapezoidProfile.Constraints armProfileConstrains = new TrapezoidProfile.Constraints(maxArmVelocity, maxArmAcceleration);
    private static final double maxMnaipulatorJointVelocity = 0; // deg/sec
    private static final double maxManipulatorJointAcceleration = 0; // deg/sec^2
    public static final TrapezoidProfile.Constraints manipulatorJointProfileConstrains = new TrapezoidProfile.Constraints(maxMnaipulatorJointVelocity, maxManipulatorJointAcceleration);

    public static final double armKs = 0; // v
    public static final double armKg = 0; // v
    public static final double armKv = 0; // v/(rad/sec)
    public static final double armKa = 0; // v/(rad/sec^2)
    public static final double manipulatorJointKg = 0; // v
    public static final double manipulatorJointKv = 0; // v/(rad/sec)

    public static final double armKp = 0;
    public static final double manipulatorJointKp = 0;

    private static final double armGearRatio = 1.0 / 64.0;
    public static final double armPositionFactor = 360 * armGearRatio; // deg/rotations
    public static final double armVelocityFactor = 360 * armGearRatio / 60; // (deg/sec)/RPM
    private static final double manipulatorJointGearRatio = 1.0 / 64.0;
    public static final double manipulatorJointPositionFactor = 360 * manipulatorJointGearRatio; // deg/rotations


}
