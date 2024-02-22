// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

    private BiConsumer<Pose2d, Double> _visionMeasurementConsumer;
    private double _lastVisionMeasurement = 0;
    private static final double VISION_MEASUREMENT_PERIOD = 0.2; // sec

    /** Creates a new LimeLight. */
    public LimeLight(BiConsumer<Pose2d, Double> visionMeasurementConsumer) {
        LimelightHelpers.setPipelineIndex(null, 0);
        _visionMeasurementConsumer = visionMeasurementConsumer;
    }

    private void addVisionMeasurement() {
        // Only add vision measurement every VISION_MEASUREMENT_PERIOD
        if (Timer.getFPGATimestamp() - _lastVisionMeasurement < VISION_MEASUREMENT_PERIOD)
            return;

        double[] result = DriverStation.getAlliance().get() == Alliance.Blue
                ? LimelightHelpers.getBotPose_wpiBlue(null)
                : LimelightHelpers.getBotPose_wpiRed(null);

        if (result.length < 7)
            return; // Drop invalid bot poses

        Pose2d visionPose = new Pose2d(result[0], result[1], new Rotation2d(Units.degreesToRadians(result[5])));
        double latency = result[6];

        _visionMeasurementConsumer.accept(visionPose, Timer.getFPGATimestamp() - (latency / 1000));

        SmartDashboard.putNumber("visionX", visionPose.getX());
        SmartDashboard.putNumber("visionY", visionPose.getY());
        SmartDashboard.putNumber("visionRotation", visionPose.getRotation().getDegrees());
        SmartDashboard.putNumber("visionLatency", latency);
    }

    @Override
    public void periodic() {
        if (LimelightHelpers.getTV(null)) {
            addVisionMeasurement();

            SmartDashboard.putBoolean("tv", true);
            SmartDashboard.putNumber("tx", LimelightHelpers.getTX(null));
        } else if (SmartDashboard.getBoolean("hasTarget", false)) {
            // Zero data when we lose the target.
            SmartDashboard.putBoolean("tv", false);
            SmartDashboard.putNumber("tx", 0.0);
        }
    }
}
