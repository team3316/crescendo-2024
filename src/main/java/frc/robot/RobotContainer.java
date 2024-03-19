
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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

        private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
                        JoysticksConstants.operatorPort);
        private final CommandPS5Controller m_driverController = new CommandPS5Controller(JoysticksConstants.driverPort);

        private final SendableChooser<Command> m_chooser;
        private final SendableChooser<SendableChooser<Command>> m_pathGroupChooser;
        private final SendableChooser<Command> m_ShootAndComChooser;
        private final SendableChooser<Command> m_4GpChooser;
        private final SendableChooser<Command> m_onlyShootChooser;
        private final SendableChooser<Command> m_nothingChooser;

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

                SmartDashboard.putBoolean("Field Relative", _fieldRelative);

                this.m_autoFactory = new AutoFactory(m_Drivetrain);

                configureNamedCommands();

                AutoBuilder.buildAutoChooser();
                this.m_chooser = new SendableChooser<Command>();
                this.m_pathGroupChooser = new SendableChooser<SendableChooser<Command>>();
                this.m_4GpChooser = new SendableChooser<Command>();
                this.m_ShootAndComChooser = new SendableChooser<Command>();
                this.m_onlyShootChooser = new SendableChooser<Command>();
                this.m_nothingChooser = new SendableChooser<Command>();
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
        }

        private void configureNamedCommands() {
                NamedCommands.registerCommand("Shoot", getAutoShootSequence());
                NamedCommands.registerCommand("Collect", getAutoCollectcommand());
                NamedCommands.registerCommand("SpinUp", getAutoSpin());
                NamedCommands.registerCommand("triger", getAutoTriggerCommand());
        }

        private void configureBindings() {
                m_driverController.options().onTrue(
                                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
                // relative mode

                m_driverController.share().onTrue(
                                new InstantCommand(m_Drivetrain::resetYaw)); // toggle field relative mode

                m_operatorController.L1().onTrue(getCollectSequence());
                m_operatorController.R2().whileTrue(getShooterSpinCommand());

                m_operatorController.povUp()
                                .whileTrue(new StartEndCommand(
                                                () -> m_Intake.setStateCommand(IntakeState.EJECT)
                                                                .alongWith(m_Manipulator.getSetStateCommand(
                                                                                ManipulatorState.EJECT))
                                                                .schedule(),
                                                () -> m_Intake.setStateCommand(IntakeState.DISABLED)
                                                                .alongWith(m_Manipulator.getSetStateCommand(
                                                                                ManipulatorState.OFF))
                                                                .schedule()));

                m_operatorController.cross().onTrue(m_Intake.setStateCommand(IntakeState.DISABLED)
                                .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.OFF)));

                m_operatorController.circle()
                                .onTrue(Commands.sequence(m_Manipulator.getSetStateCommand(ManipulatorState.AMP),
                                                new WaitCommand(3), m_Manipulator
                                                                .getSetStateCommand(ManipulatorState.OFF)));
                m_operatorController.povDown()
                                .onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.COLLECT));

                m_operatorController.square().onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.AMP)
                                .alongWith(m_Manipulator.getMoveNoteToPositionCommand(NotePosition.AMP)));

                m_driverController.square().onTrue(m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.PRE_CLIB));
                m_driverController.triangle().whileTrue(m_ArmWristSuperStructure.getClimbCommand());
                m_driverController.R1().onTrue(getShooterTriggerCommand());

        }

        private Command getShooterTriggerCommand() {
                Command sequence = new ConditionalCommand(
                                Commands.sequence(
                                                new WaitUntilCommand(
                                                                () -> m_Shooter.isAtTargetVelocity() && m_Shooter
                                                                                .getShooterState() == ShooterState.ON),
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


        private Command getCollectSequence() {
                Command sequence = Commands.sequence(
                                m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.COLLECT)
                                                .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.COLLECT)),
                                m_Intake.setStateCommand(IntakeState.COLLECTING),
                                Commands.deadline(new WaitUntilCommand(() -> m_Manipulator.hasNoteSwitch()),
                                                Commands.sequence(new WaitUntilCommand(() -> m_Intake.isNoteInIntake()),
                                                                new WaitUntilCommand(() -> !m_Intake.isNoteInIntake()),
                                                                m_Manipulator.getSetStateCommand(
                                                                                ManipulatorState.SLOW_COLLECT)
                                                                                .alongWith(m_Intake.setStateCommand(
                                                                                                IntakeState.SLOW_COLLECT)))),
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

        private Command getAutoCollectcommand() {
                Command sequence = Commands.sequence(
                                m_Shooter.getSetStateCommand(ShooterState.OFF),
                                m_ArmWristSuperStructure.getSetStateCommand(ArmWristState.COLLECT)
                                                .alongWith(m_Manipulator.getSetStateCommand(ManipulatorState.COLLECT)),
                                m_Intake.setStateCommand(IntakeState.COLLECTING),
                                new WaitUntilCommand(() -> m_Manipulator.hasNoteSwitch()),
                                m_Manipulator.getSetStateCommand(ManipulatorState.OFF),
                                m_Intake.setStateCommand(IntakeState.DISABLED));

                return sequence;

        }

        private Command getAutoShootSequence() {
                return Commands.sequence(
                                m_Shooter.getSetStateCommand(ShooterState.ON),
                                new WaitUntilCommand(() -> m_Shooter.isAtTargetVelocity()),
                                m_Manipulator.getSetStateCommand(ManipulatorState.TO_SHOOTER),
                                new WaitCommand(0.5),
                                m_Manipulator.getSetStateCommand(ManipulatorState.OFF)
                                                .alongWith(m_Shooter
                                                                .getSetStateCommand(ShooterState.OFF)));
        }

        private Command getAutoSpin() {
                return new InstantCommand(() -> m_Shooter.getSetStateCommand(ShooterState.ON));
        }

        private Command getAutoTriggerCommand() {
                Command sequence = Commands.sequence(
                                m_Manipulator.getSetStateCommand(ManipulatorState.TO_SHOOTER),
                                new WaitCommand(0.2),
                                m_Shooter.getSetStateCommand(ShooterState.OFF)
                                                .andThen(m_Manipulator.getSetStateCommand(
                                                                ManipulatorState.OFF)));
                return sequence;
        }

        private void initChooser() {

                SmartDashboard.putData("autoChooser", m_pathGroupChooser);
              
                m_pathGroupChooser.addOption("4gp", m_4GpChooser);
                m_pathGroupChooser.addOption("shoot and leave", m_ShootAndComChooser);
                m_pathGroupChooser.addOption("only shoot", m_onlyShootChooser);
                m_pathGroupChooser.addOption("nothing", m_nothingChooser);

                m_pathGroupChooser.onChange((chooser)->setChooser(chooser));
  

                m_4GpChooser.addOption("4gpAMP", null);
                m_4GpChooser.addOption("4gpMID",  m_autoFactory.createAuto("4_gp"));
                m_4GpChooser.addOption("4gpSOURCE", null);
                m_4GpChooser.addOption("4gpAMPcenter", null);
                m_4GpChooser.addOption("4gpMIDcenter", null);
                m_4GpChooser.addOption("4gpSOURCEcenter", null);

                m_ShootAndComChooser.addOption("source shoot and exit", m_autoFactory.createAuto("source_Shoot_Com"));
                m_ShootAndComChooser.addOption("amp shoot and exit",m_autoFactory.createAuto("amp_Shoot_Com"));

                m_onlyShootChooser.addOption("only shoot", getAutoShootSequence());

                m_nothingChooser.addOption("nothing", new InstantCommand());


                SmartDashboard.putData("Auto Chooser PLACE BY DRIVERS!", m_chooser);


                m_chooser.addOption("4_gp", m_autoFactory.createAuto("4_gp"));

              


        }


        public void setChooser(SendableChooser chooser){
                SmartDashboard.putData("variant chooser", chooser);
                
                
                
        }
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return m_pathGroupChooser.getSelected().getSelected();
        }
}
