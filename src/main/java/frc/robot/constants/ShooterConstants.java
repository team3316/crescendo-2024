package frc.robot.constants;


public class ShooterConstants {
    public static final int leaderUpLeftPort = 14;
    public static final int followerUpRightPort = 15;
    public static final int followerDownLeftPort = 12;
    public static final int followerDownRightPort = 13;


    public static final double SparkFlexUnShootingVelocity = 0; // rpm
    public static final double SparkFlexShootingVelocity = 1; // rpm

    public static final double kpShooter = 0; // percentage/rpm
    public static final double kfShooter = 0.75 / 5090; // percentage/rpm 

    public static final double shootingTime = 2; // seconds
}
