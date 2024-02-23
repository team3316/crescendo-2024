
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.AutoFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveSysidCommands;

public class RobotContainer {

    private final Drivetrain m_Drivetrain = new Drivetrain();
    // private final Arm m_Arm = new Arm();
    private final Manipulator m_Manipulator = new Manipulator();
    private final Shooter m_Shooter = new Shooter();
    private final Intake m_Intake = new Intake();
    // private final Climber m_Climber = new Climber(() ->
    // Rotation2d.fromDegrees(m_Drivetrain.getRoll()));

    private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
            JoysticksConstants.operatorPort);
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(JoysticksConstants.driverPort);

    private final SendableChooser<Command> chooser;
    private final AutoFactory _autoFactory;

    private boolean _fieldRelative = true;

    public RobotContainer() {
        m_Drivetrain.setDefaultCommand(new RunCommand(() -> m_Drivetrain.drive(
                m_driverController.getLeftY() *
                        SwerveModuleConstants.driveFreeSpeedMetersPerSecond,
                m_driverController.getLeftX() *
                        SwerveModuleConstants.driveFreeSpeedMetersPerSecond,
                m_driverController.getCombinedAxis() *
                        DrivetrainConstants.maxRotationSpeedRadPerSec,
                _fieldRelative), m_Drivetrain));
        this._autoFactory = new AutoFactory(m_Drivetrain);
        this.chooser = AutoBuilder.buildAutoChooser();
        initChooser();
        // Configure the trigger bindings
        configureBindings();
    }

    public void stop() {
        m_Drivetrain.disabledInit();
        // m_Arm.stop();
        m_Intake.stop();
        m_Manipulator.stop();
        m_Shooter.stop();
        // m_Climber.stop();
    }

    private void configureBindings() {
        m_driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
        // relative mode

        m_driverController.share().onTrue(
                new InstantCommand(m_Drivetrain::resetYaw)); // toggle field relative mode

        m_driverController.L1().onTrue(getCollectSequence());
        m_driverController.R1().onTrue(getShootSequence());
        m_driverController.povDown().onTrue(m_Intake.setStateCommand(IntakeState.EJECT));
        // m_buttonController.cross().whileTrue(getAMPSequence());
        // m_buttonController.square().onTrue(m_Arm.getSetStateCommand(ArmState.UNDER_CHAIN));
        // m_buttonController.triangle().onTrue(getClimbSequence());
        /*
         * driver should press this before cross to save time, but cross still includes
         * arm to amp in case of mistake
         */
        //// m_buttonController.circle().onTrue(m_Arm.getSetStateCommand(ArmState.AMP));
    }

    private Command getCollectSequence() {
        Command sequence = Commands.sequence(
                /*
                 * m_Arm.getSetStateCommand(ArmState.COLLECT)
                 * .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.COLLECT)),
                 */
                m_Manipulator.getSetStateCommand(ManipulatorState.COLLECT),
                m_Intake.setStateCommand(IntakeState.COLLECTING),
                new WaitUntilCommand(() -> m_Manipulator.hasNoteSwitch()),
                m_Intake.setStateCommand(IntakeState.DISABLED)
                        .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.OFF)));
        return sequence;
    }

    private Command getShootSequence() {
        Command sequence = new ConditionalCommand(
                Commands.sequence(
                        // m_Arm.getSetStateCommand(ArmState.COLLECT), // in case of moving to
                        // amp and then regretting
                        m_Shooter.getSetStateCommand(ShooterState.ON),
                        new WaitUntilCommand(() -> m_Shooter.isAtTargetVelocity()),
                        // new WaitCommand(0.2),
                        m_Manipulator.getSetStateCommand(ManipulatorState.TO_SHOOTER),
                        new WaitCommand(2),
                        m_Manipulator.getSetStateCommand(ManipulatorState.OFF)
                                .alongWith(m_Shooter
                                        .getSetStateCommand(ShooterState.OFF))),
                new InstantCommand(),
                () -> m_Manipulator.hasNoteSwitch());
        return sequence;
    }

    // private Command getAMPSequence() {
    // Command start = m_Arm.getSetStateCommand(ArmState.AMP)
    // .andThen(m_Manipulator.getSetStateCommand(ManipulatorState.AMP));
    // Command end = m_Manipulator.getSetStateCommand(ManipulatorState.OFF)
    // .andThen(m_Arm.getSetStateCommand(ArmState.COLLECT));
    // return new StartEndCommand(() -> {
    // start.schedule();
    // }, () -> {
    // start.cancel();
    // end.schedule();
    // });
    // }

    // private Command getClimbSequence() {
    // return
    // m_Arm.getSetStateCommand(ArmState.TRAP).andThen(m_Climber.getClimbCommand());
    // }

    private void initChooser() {
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
