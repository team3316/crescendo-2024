// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  LimeLight m_LimeLight;
  Drivetrain m_Drivetrain;
  private boolean _fieldRelative = true;
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);
  
  public RobotContainer() {
    m_Drivetrain = new Drivetrain();
    m_LimeLight = new LimeLight();
   

     m_Drivetrain.setDefaultCommand(new RunCommand(() -> m_Drivetrain.drive(
                m_driverController.getLeftY() *
                        SwerveModuleConstants.freeSpeedMetersPerSecond,
                m_driverController.getLeftX() *
                        SwerveModuleConstants.freeSpeedMetersPerSecond,
                m_driverController.getCombinedAxis() *
                        DrivetrainConstants.maxRotationSpeedRadPerSec,
                _fieldRelative), m_Drivetrain));

    
    // Configure the trigger bindings
    configureBindings();
  }

  

  private void configureBindings() {
     
                m_driverController.triangle().whileTrue(new RunCommand(() -> m_Drivetrain.drive(
                                m_driverController.getLeftY() *
                                                SwerveModuleConstants.freeSpeedMetersPerSecond,
                                m_driverController.getLeftX() *
                                                SwerveModuleConstants.freeSpeedMetersPerSecond,
                                m_Drivetrain.getRotByVision(Math.toRadians(m_LimeLight.getXAngle()), m_LimeLight.hasTarget()),
                                _fieldRelative), m_Drivetrain));

                m_driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
        // relative mode

        m_driverController.share().onTrue(
                new InstantCommand(m_Drivetrain::resetYaw)); // toggle field relative mode
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}