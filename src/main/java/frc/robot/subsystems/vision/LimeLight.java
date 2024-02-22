// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class LimeLight extends SubsystemBase {
    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
        LimelightHelpers.setPipelineIndex(null, 0);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(null);
    }

    public double getXAngle() {
        return LimelightConstants.AlignLimeLightToPigeonPhase * LimelightHelpers.getTX(null);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hasTarget", hasTarget());
        if (hasTarget())
            SmartDashboard.putNumber("getXAngle", getXAngle());
    }
}
