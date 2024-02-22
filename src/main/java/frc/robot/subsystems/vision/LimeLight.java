// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
        LimelightHelpers.setPipelineIndex(null, 0);
    }

    @Override
    public void periodic() {
        if (LimelightHelpers.getTV(null)) {
            SmartDashboard.putBoolean("tv", true);
            SmartDashboard.putNumber("tx", LimelightHelpers.getTX(null));
        } else if (SmartDashboard.getBoolean("hasTarget", false)) {
            // Zero data when we lose the target.
            SmartDashboard.putBoolean("tv", false);
            SmartDashboard.putNumber("tx", 0.0);
        }
    }
}
