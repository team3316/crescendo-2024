package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.motors.PIDFGains;

/**
 * DrivetrainConstants
 */
public class DrivetrainConstants {
        public static class SwerveModuleConstants {
                public static final double krakenFreeSpeed = 6000; // RPM
                private static final double driveRatio = 1.0 / 6.12;
                private static final double steeringRatio = 1.0 / 12.8;
                private static final double wheelDiameterMeters = 3.85 * 2.54 / 100; // 4 inches in meters

                public static final double drivePositionConversionFactor = driveRatio * wheelDiameterMeters * Math.PI; // m
                                                                                                                       // /
                                                                                                                       // rotation

                public static final double steeringPositionConversionFactor = steeringRatio * 360; // degrees / rotation
                public static final double steeringVelocityConversionFactor = steeringPositionConversionFactor / 60; // degrees
                                                                                                                     // /
                                                                                                                     // (rotation
                                                                                                                     // *
                                                                                                                     // seconds/minute)

                public static final double driveFreeSpeedMetersPerSecond = krakenFreeSpeed / 60 * drivePositionConversionFactor;

                public static final double driveSpeedLimit = 0.25;

                // TODO: update pidf gains
                public static final double driveKp = 1; // voltage / (m/sec)
                public static final double driveKd = 0; // voltage / (m/sec^2)
                public static final double driveKv = 12/driveFreeSpeedMetersPerSecond; // voltage / (m/sec)
                public static final double steeringKp = 0.0124; // in 1 / wheel degrees
                
                public final Translation2d position;
                public final int idDrive;
                public final PIDFGains driveGains = new PIDFGains(driveKp, 0, driveKd, driveKv);
                public final int idSteering;
                public final PIDFGains steeringGains = new PIDFGains(steeringKp);
                public final double cancoderZeroAngle;
                public final int canCoderId;

                public SwerveModuleConstants(
                                Translation2d position,
                                int idDrive,
                                int idSteering,
                                double cancoderZeroAngle,
                                int canCoderId) {

                        this.position = position;
                        this.idDrive = idDrive;
                        this.idSteering = idSteering;
                        this.cancoderZeroAngle = cancoderZeroAngle;
                        this.canCoderId = canCoderId;
                }
        }

        public static final double maxRotationSpeedRadPerSec = 11.5;

        public static final double frontWheelDistMeters = 0.7403;
        public static final double sideWheelDistMeters = 0.4253;

        public final static double cancoderTROffset = 21.7;
        public final static double cancoderTLOffset = 283.8;
        public final static double cancoderBROffset = 212.6;        
        public final static double cancoderBLOffset = 143.4;
        
        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(
                        new Translation2d(sideWheelDistMeters / 2, frontWheelDistMeters / 2), 17, 2,
                        cancoderTLOffset, 21);

        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(
                        new Translation2d(sideWheelDistMeters / 2, -frontWheelDistMeters / 2), 16, 1,
                        cancoderTROffset, 20);

        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(
                        new Translation2d(-sideWheelDistMeters / 2, frontWheelDistMeters / 2), 19, 4,
                        cancoderBLOffset, 23);

        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(
                        new Translation2d(-sideWheelDistMeters / 2, -frontWheelDistMeters / 2), 18, 3,
                        cancoderBROffset, 22);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
                        TLModule.position, BRModule.position, BLModule.position);

        public static final int pigeonId = 24;

        public static final Rotation2d installAngle = Rotation2d.fromDegrees(0);
        public static final Rotation2d collectAngle = Rotation2d.fromDegrees(90);

        public static final double driveCurrentLimit = 60;

}