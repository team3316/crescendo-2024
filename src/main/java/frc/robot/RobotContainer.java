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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain m_drivetrain = new Drivetrain();
  private boolean _fieldRelative = true;
  private final CommandPS5Controller _driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);
  public RobotContainer() {
    m_Drivetrain = new Drivetrain();

     m_Drivetrain.setDefaultCommand(new RunCommand(() -> m_Drivetrain.drive(
                _driverController.getLeftY() *
                        SwerveModuleConstants.driveFreeSpeedMetersPerSecond,
                _driverController.getLeftX() *
                        SwerveModuleConstants.driveFreeSpeedMetersPerSecond,
                _driverController.getCombinedAxis() *
                        DrivetrainConstants.maxRotationSpeedRadPerSec,
                _fieldRelative), m_Drivetrain));

    
    // Configure the trigger bindings
    configureBindings();
  }

  

  private void configureBindings() {
     _driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
        // relative mode

        _driverController.share().onTrue(
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
