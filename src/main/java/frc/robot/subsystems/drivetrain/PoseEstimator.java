// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.vision.BotPose;
import frc.robot.subsystems.vision.LimeLight;

public class PoseEstimator extends SubsystemBase {

      // maximum allowed deviation in meters between current pose and vision pose in
    // order to accept the vision reading as valid.
    private static final double visionMeasurementRejectionThreshold = 2.0;
        private double lastVisionTimestamp = -1;
    Drivetrain drivetrain;        
    LimeLight limeLight;

    private final double xTolerance = 0.2; // in meters
    private final double yTolerance = 0.1; // in meters
    private final double tTolerance = 10; // in degrees

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    public PoseEstimator(Drivetrain drivetrain, LimeLight limeLight) {
        this.limeLight = limeLight;
        this.drivetrain = drivetrain;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.kinematics,
                drivetrain.getRotation2d(),
                drivetrain.getSwerveModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

      
    }

    private void addValidVisionMeasurement(BotPose robotPose) {
        if (robotPose == null) {
            return;
        }

        if (getCurrentPose().minus(robotPose.getPose()).getTranslation()
                .getNorm() > visionMeasurementRejectionThreshold) {
            return;
        }
        SmartDashboard.putNumber("poseX", robotPose.getPose().getX());
        SmartDashboard.putNumber("posey", robotPose.getPose().getY());
        SmartDashboard.putNumber("poseRot", robotPose.getPose().getRotation().getDegrees());

        lastVisionTimestamp = robotPose.getTimestamp();
        poseEstimator.addVisionMeasurement(robotPose.getPose(), robotPose.getTimestamp());
    }

    @Override
    public void periodic() {
        // Update pose estimator with the best visible target
        addValidVisionMeasurement(limeLight.getBotPose());

        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
                drivetrain.getRotation2d(),
                drivetrain.getSwerveModulePositions());

        field2d.setRobotPose(getCurrentPose());
        SmartDashboard.putData("field", field2d);
    }
    

    private boolean isVisionSynced() {
        return (Timer.getFPGATimestamp() - lastVisionTimestamp) < 5;
    }


    
    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                drivetrain.getRotation2d(),
                drivetrain.getSwerveModulePositions(),
                newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

   

}