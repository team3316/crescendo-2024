package frc.robot.constants;


public class ShooterConstants {
    public static final int leaderUpLeftPort = 12;
    public static final int followerUpRightPort = 13;
    public static final int followerDownLeftPort = 12;
    public static final int followerDownRightPort = 13;

    public static final double SparkFlexUnShootingVelocity = 0; // mps
    public static final double SparkFlexShootingVelocity = 7; // mps

    private static final double wheelDiameter = 4 * 0.0254; // 4 inches in meters
    public static final double positionFactor = wheelDiameter; // meters / rotation
    public static final double velocityFactor = wheelDiameter / 60; // (meters / second) / RPM

    public static final double kpShooter = 0.0002 / velocityFactor; // percentage/rpm
    public static final double kfShooter = 0.75 / (4874 * velocityFactor); // percentage/(meters per second)
    public static final double shootingTime = 2; // seconds
}
