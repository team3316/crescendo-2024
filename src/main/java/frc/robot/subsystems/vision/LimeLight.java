// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class LimeLight extends SubsystemBase {
    NetworkTable limeLightTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipeLine;
    private NetworkTableEntry LEDs;

    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code

        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
        hasTarget = limeLightTable.getEntry("tv");
        pipeLine = limeLightTable.getEntry("pipeline");
        LEDs = limeLightTable.getEntry("ledMode");
        SmartDashboard.putNumber("tx", tx.getDouble(52));
        setPipeLine(0);
    }

    public boolean hasTarget() {
        return hasTarget.getDouble(0) == 1;
    }

    public double getXAngle() {
        return LimelightConstants.AlignLimeLightToPigeonPhase * tx.getDouble(0);
    }

    public void setPipeLine(double id) {
        pipeLine.setNumber(id);
    }

    public void forceLEDsOff(boolean off) {
        LEDs.setNumber(off ? LimelightConstants.LEDsForceOff : LimelightConstants.LEDsByPipeline);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hasTarget", hasTarget());
        if (hasTarget())
            SmartDashboard.putNumber("getXAngle", getXAngle());
    }
}
