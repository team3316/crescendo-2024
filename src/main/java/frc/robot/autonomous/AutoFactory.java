// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** Add your docs here. */
public class AutoFactory {

    AutoBuilder _autoBuilder;

    public AutoFactory(Drivetrain drivetrain){
        _autoBuilder = new AutoBuilder();
        AutoBuilder.configureHolonomic(
        drivetrain::getPose, // Robot pose supplier
        drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        drivetrain::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        drivetrain::shouldFlipPath,
        drivetrain // Reference to this subsystem to set requirements
        );
    }
    public Command createAuto(String pathName){
    return new PathPlannerAuto(pathName);
}
    
}
