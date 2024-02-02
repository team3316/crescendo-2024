// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class LimeLight extends SubsystemBase {
    NetworkTable limeLightTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipeLine;
    private NetworkTableEntry LEDs;
    private double hDiff = 0;

    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
   
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
        ty = limeLightTable.getEntry("ty");
        hasTarget = limeLightTable.getEntry("tv");
        pipeLine = limeLightTable.getEntry("pipeline");
        LEDs = limeLightTable.getEntry("ledMode");
        SmartDashboard.putNumber("tx", tx.getDouble(52));
             setPipeLine(0);
    }

    public boolean hasTarget() {
        return hasTarget.getDouble(0) == 1;
    }

    private double getXAngle() {
        return LimelightConstants.AlignLimeLightToPigeonPhase * tx.getDouble(0);
    }

    private double getYAngle() {
        return ty.getDouble(0);
    }

    public double getFieldYMeters() {
        return this.hDiff * Math.sin(Math.toRadians(this.getXAngle()))
                / Math.tan(Math.toRadians(this.getYAngle()));
    }

    public double getFieldXMeters() {
        return this.hDiff * Math.cos(Math.toRadians(this.getXAngle()))
                / Math.tan(Math.toRadians(this.getYAngle()));
    }

    public void setPipeLine(double id) {
        pipeLine.setNumber(id);
    }

    public void setTargetHeightDiff(double hDiff) {
        this.hDiff = hDiff;
    }

    public void forceLEDsOff(boolean off) {
        LEDs.setNumber(off ? LimelightConstants.LEDsForceOff : LimelightConstants.LEDsByPipeline);
    }

public BotPose getBotPose() {
        if (!hasTarget())
            return null;

        String entryName = "botpose_wpiblue";

        double[] poseComponents = limeLightTable.getEntry(entryName).getDoubleArray(new double[6]);
        if (poseComponents.length == 0)
            return null;
        double latency = limeLightTable.getEntry("tl").getDouble(0) + limeLightTable.getEntry("cl").getDouble(0);

        return new BotPose(new Pose2d(
                poseComponents[0],
                poseComponents[1],
                new Rotation2d(Math.toRadians(poseComponents[5]))),
                Timer.getFPGATimestamp() - (latency / 1000.0));

    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hasTarget",hasTarget() );

        SmartDashboard.putNumber("xLength", this.getFieldXMeters());
        SmartDashboard.putNumber("yLength", this.getFieldYMeters());
    }
}
