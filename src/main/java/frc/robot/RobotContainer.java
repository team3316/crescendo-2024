
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.AutoFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Manipulator.NotePosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.arm.ArmWristSuperStructure;
import frc.robot.subsystems.arm.ArmWristSuperStructure.ArmWristState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveSysidCommands;
import frc.robot.subsystems.vision.LimeLight;

public class RobotContainer {

    private final Drivetrain m_Drivetrain = new Drivetrain();
    private final ArmWristSuperStructure m_ArmWristSuperStructure = new ArmWristSuperStructure();
    private final Manipulator m_Manipulator = new Manipulator();
    private final Shooter m_Shooter = new Shooter();
    private final Intake m_Intake = new Intake();
    private final LimeLight m_limeLight = new LimeLight();
    private final Climber m_Climber = new Climber(() -> Rotation2d.fromDegrees(m_Drivetrain.getRoll()));

    private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
            JoysticksConstants.operatorPort);
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(JoysticksConstants.driverPort);

    private final SendableChooser<Command> m_chooser;
    private final AutoFactory m_autoFactory;

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

        m_Intake.setDefaultCommand(m_Intake.setStateCommand(IntakeState.DISABLED));
        m_Manipulator.setDefaultCommand(m_Manipulator.getSetStateCommand(ManipulatorState.OFF));
        
        SmartDashboard.putBoolean("Field Relative", _fieldRelative);

        this.m_autoFactory = new AutoFactory(m_Drivetrain);
        NamedCommands.registerCommand("Shoot", getAutoShootSequence());
        NamedCommands.registerCommand("Collect", getCollectSequence());

        AutoBuilder.buildAutoChooser();
        this.m_chooser = new SendableChooser<Command>();
        initChooser();
        // Configure the trigger bindings
        configureBindings();
    }

    public void stop() {
        m_Drivetrain.disabledInit();
        m_ArmWristSuperStructure.stop();
        m_Intake.stop();
        m_Manipulator.stop();
        m_Shooter.stop();
        m_Climber.stop();
    }

    private void configureBindings() {
        m_driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
        // relative mode

        m_driverController.share().onTrue(
                new InstantCommand(m_Drivetrain::resetYaw)); // toggle field relative mode

        m_operatorController.L1().onTrue(getCollectSequence());
        m_operatorController.R1().onTrue(getShooterTriggerCommand());
        m_operatorController.R2().whileTrue(getShooterSpinCommand());
        // m_operatorController.circle().onTrue(m_Intake.setStateCommand(IntakeState.EJECT));
        m_operatorController.circle()
                .onTrue(Commands.sequence(m_Manipulator.getSetStateCommand(ManipulatorState.AMP),
                        new WaitCommand(3), m_Manipulator
                                .getSetStateCommand(ManipulatorState.OFF)));
        m_operatorController.povDown()
                .onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.COLLECT));
        m_driverController.povRight()
                .onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.UNDER_CHAIN));
        m_driverController.povUp().onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.ALIGN));
        m_driverController.povLeft().onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.TRAP)
                .alongWith(new WaitCommand(2).andThen(
                        m_Manipulator.getMoveNoteToPositionCommand(NotePosition.TRAP))));
        m_operatorController.triangle().onTrue(m_Climber.getClimbCommand());
        m_operatorController.L2()
                .onTrue(Commands.sequence(m_Manipulator.getSetStateCommand(ManipulatorState.TRAP),
                        new WaitCommand(3),
                        m_Manipulator.getSetStateCommand(ManipulatorState.OFF)));
        m_operatorController.square().onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.AMP)
                .alongWith(Commands.sequence(new WaitCommand(1),
                        m_Manipulator.getMoveNoteToPositionCommand(NotePosition.AMP))));

        m_Climber.setDefaultCommand(
                new RunCommand(() -> m_Climber.setPercentage(m_operatorController.getLeftY() * 0.2,
                        m_operatorController.getRightY() * 0.2), m_Climber));// stupid climb
    }

    private Command getCollectSequence() {
        Command sequence = Commands.sequence(
                m_Shooter.getSetStateCommand(ShooterState.OFF),
                m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.COLLECT)
                        .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.COLLECT)),
                m_Intake.setStateCommand(IntakeState.COLLECTING),
                new WaitUntilCommand(() -> m_Manipulator.hasNoteSwitch()),
                m_Intake.setStateCommand(IntakeState.EJECT)
                        .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.OFF)),
                new WaitCommand(2),
                m_Intake.setStateCommand(IntakeState.DISABLED));
        return new ConditionalCommand(new InstantCommand(), sequence, m_Manipulator::hasNoteSwitch);
    }

    private Command getShooterSpinCommand() {
        return new StartEndCommand(() -> m_Shooter.getSetStateCommand(ShooterState.ON).schedule(),
                () -> m_Shooter.getSetStateCommand(ShooterState.OFF).schedule());
    }

    private Command getShooterTriggerCommand() {
        Command sequence = new ConditionalCommand(
                Commands.sequence(
                        new WaitUntilCommand(() -> m_Shooter.isAtTargetVelocity()),
                        m_Manipulator.getSetStateCommand(ManipulatorState.TO_SHOOTER),
                        new WaitCommand(2),
                        m_Shooter.getSetStateCommand(ShooterState.OFF)
                                .andThen(m_Manipulator.getSetStateCommand(
                                        ManipulatorState.OFF)))
                        .alongWith(
                                new WaitCommand(2),
                                m_Intake.setStateCommand(IntakeState.DISABLED)),
                new InstantCommand(),
                () -> m_Manipulator.hasNoteSwitch());
        return sequence;
    }

    private Command getAutoShootSequence() {
        return Commands.sequence(
                m_Shooter.getSetStateCommand(ShooterState.ON),
                new WaitUntilCommand(() -> m_Shooter.isAtTargetVelocity()),
                m_Manipulator.getSetStateCommand(ManipulatorState.TO_SHOOTER),
                new WaitCommand(2),
                m_Manipulator.getSetStateCommand(ManipulatorState.OFF)
                        .alongWith(m_Shooter
                                .getSetStateCommand(ShooterState.OFF)));
    }

    private void initChooser() {
        SmartDashboard.putData("Auto Chooser PLACE BY DRIVERS!", m_chooser);
        // basic
        m_chooser.addOption("mid shoot and exit", m_autoFactory.createAuto("MID_Shoot_Com"));
        m_chooser.addOption("right shoot and exit", m_autoFactory.createAuto("LEFT_Shoot_Com"));// oppisate in
                                                                                                // path, no idea
                                                                                                // how changing
                                                                                                // name will
                                                                                                // effect PP
        m_chooser.addOption("left shoot and exit", m_autoFactory.createAuto("RIGHT_Shoot_Com"));// TODO: fix
                                                                                                // befor ISR3

        // "orbit"
        m_chooser.addOption("mid 4 notes", m_autoFactory.createAuto("MID_ORBIT"));
        m_chooser.addOption("left 4 notes", m_autoFactory.createAuto("LEFT_ORBIT"));
        m_chooser.addOption("right 4 notes", m_autoFactory.createAuto("RIGHT_ORBIT"));

        // only shoot
        m_chooser.addOption("only shoot", getAutoShootSequence());

        // nothing
        m_chooser.addOption("nothing", new InstantCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
