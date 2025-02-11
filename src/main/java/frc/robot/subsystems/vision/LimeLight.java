// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class LimeLight extends SubsystemBase {
    private static final boolean UPDATE_DASHBOARD = true;

    NetworkTable limeLightTable;
    private NetworkTableEntry ta;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipeLine;
    private NetworkTableEntry LEDs;
    private double hDiff = 0;

    private PIDController distanceController;
    private PIDController angleController;

    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
   
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        ta = limeLightTable.getEntry("ta");
        tx = limeLightTable.getEntry("tx");
        ty = limeLightTable.getEntry("ty");
        hasTarget = limeLightTable.getEntry("tv");
        pipeLine = limeLightTable.getEntry("pipeline");
        LEDs = limeLightTable.getEntry("ledMode");
        //SmartDashboard.putNumber("Limelight/tx", tx.getDouble(52));
             setPipeLine(0);

        distanceController = new PIDController(LimelightConstants.yKp, 0, 0);
        angleController = new PIDController(LimelightConstants.thetaKp, 0, 0);
    }
    public double getArea(){
  
        return this.ta.getDouble(-1);
        
    }
    public boolean hasTarget() {
        return hasTarget.getDouble(0) == 1;
    }

    public double getXAngle() {
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

    private double getDistanceFromTarget() {
        return LimelightConstants.speakerTargetHeight / Math.tan(Math.toRadians(getYAngle() + LimelightConstants.limelightMountingAngle));
    }

    public double getDistanceOutput() {
        if (hasTarget()) {
            return distanceController.calculate(getDistanceFromTarget(), LimelightConstants.distanceSetpoint);
        }
        return 0;
    }

    public double getAngleOutput() {
        if (hasTarget()) {
            return -angleController.calculate(getXAngle(), 0);
        }
        return 0;
    }
    
    @Override
    public void periodic() {
        if (UPDATE_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight/hasTarget",hasTarget());
            SmartDashboard.putNumber("Limelight/distance from target", getDistanceFromTarget());
            SmartDashboard.putNumber("Limelight/tx", getXAngle());
            SmartDashboard.putNumber("Limelight/ty", getYAngle());
        }
    }
}
