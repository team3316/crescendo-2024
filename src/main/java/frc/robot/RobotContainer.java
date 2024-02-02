// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain m_Drivetrain = new Drivetrain();
  private boolean _fieldRelative = true;
  private final CommandPS5Controller _driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);

  private Arm m_Arm = new Arm();
  private Manipulator m_Manipulator = new Manipulator();
  private Shooter m_Shooter = new Shooter();
  private Intake m_Intake = new Intake();
  private CommandPS5Controller m_buttonController;

  public RobotContainer() {
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

  public void stop() {
    m_Drivetrain.disabledInit();
  }

  private void configureBindings() {
    _driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
        // relative mode

    _driverController.share().onTrue(
                new InstantCommand(m_Drivetrain::resetYaw)); // toggle field relative mode
    
    m_buttonController.L1().onTrue(getCollectSequence());
    m_buttonController.R1().onTrue(getShootSequence());
    m_buttonController.cross().whileTrue(getAMPSequence());

    /* driver should press this before cross to save time, but cross still includes arm to amp in case of mistake */
    m_buttonController.circle().onTrue(m_Arm.getSetStateCommand(ArmState.AMP));
  }

  private Command getCollectSequence() {
    return Commands.sequence(
      m_Arm.getSetStateCommand(ArmState.COLLECT).alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.COLLECT)),
      m_Intake.setStateCommand(IntakeState.COLLECTING),
      new WaitUntilCommand(() -> m_Manipulator.hasNoteSwitch()),
      m_Intake.setStateCommand(IntakeState.DISABLED).alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.OFF))
    );
  }

  private Command getShootSequence() {
    return new ConditionalCommand(
      Commands.sequence(
        m_Arm.getSetStateCommand(ArmState.COLLECT), // in case of moving to amp and then regretting
        m_Shooter.getSetStateCommand(ShooterState.ON),
        new WaitUntilCommand(() -> m_Shooter.isAtTargetVelocity()),
        m_Manipulator.getSetStateCommand(ManipulatorState.TOSHOOTER),
        new WaitCommand(1), // arbitrary time
        m_Manipulator.getSetStateCommand(ManipulatorState.OFF).alongWith(m_Shooter.getSetStateCommand(ShooterState.OFF))),
      new InstantCommand(),
      () -> m_Manipulator.hasNoteSwitch()
    );
  }

  private Command getAMPSequence() {
    Command start = m_Arm.getSetStateCommand(ArmState.AMP).andThen(m_Manipulator.getSetStateCommand(ManipulatorState.AMP));
    Command end = m_Manipulator.getSetStateCommand(ManipulatorState.OFF).andThen(m_Arm.getSetStateCommand(ArmState.COLLECT));
    return new StartEndCommand(() -> {start.schedule();}, () -> {start.cancel(); end.schedule();});
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
