package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.vision.LimelightHelpers;

/**
 * Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private static final boolean UPDATE_TELEMETRY = false;
    private static final boolean UPDATE_DASHBOARD = false;

    private SwerveModule[] _modules;

    private PigeonIMU _pigeon;

    private SwerveDrivePoseEstimator _estimator;

    private StructLogEntry<Pose2d> m_poseLog;

    private StructArrayLogEntry<SwerveModuleState> m_modulesCurrentStateLog;
    private StructArrayLogEntry<SwerveModuleState> m_modulesWantedStateLog;

    private static PIDController angleController;

    public Drivetrain() {
        this._modules = new SwerveModule[] {
                new SwerveModule(DrivetrainConstants.TRModule),
                new SwerveModule(DrivetrainConstants.TLModule),
                new SwerveModule(DrivetrainConstants.BRModule),
                new SwerveModule(DrivetrainConstants.BLModule)
        };

        _pigeon = new PigeonIMU(DrivetrainConstants.pigeonId);

        initPoseEstimator();

        initTelemetry();

        angleController = new PIDController(LimelightConstants.angleKp, 0, 0);
        angleController.setTolerance(LimelightConstants.angleTol);
        angleController.setSetpoint(0);

        calibrateSteering();

    }

    private void initPoseEstimator() {
        /**
         * These are the default standard deviations provided by the
         * SwerveDrivePoseEstimator class.
         * 
         * TODO: Update StdDevs according to the documentation instructions:
         * > When incorporating AprilTag poses, make the vision heading standard
         * > deviation very large, make the gyro heading standard deviation small, and
         * > scale the vision x and y standard deviation by distance from the tag.
         */
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

        _estimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.kinematics,
                getRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(), // initial pose to be supplied through `resetPose` during autonomous
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    /************************
     * Drivetrain Interface *
     ************************/

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        fieldRelative = fieldRelative && this._pigeon.getState() == PigeonState.Ready;
        SmartDashboard.putBoolean("Field Relative", fieldRelative);

        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        var moduleStates = DrivetrainConstants.kinematics.toSwerveModuleStates(speeds);

        setDesiredStates(moduleStates);
    }

    /**
     * Drives by X and Y inputs, maintaining given target angle given from vision
     * target
     * 
     * @param xSpeed        speed in the right direction (negative is left)
     * @param ySpeed        speed in the front direction (negative is back)
     * @param fieldRelative treat x-y relative odometry pose
     */
    public void driveByVision(double xSpeed, double ySpeed, boolean fieldRelative) {
        // If vision doesn't have a target, or we reached our setpoint, don't rotate.
        var rot = !LimelightHelpers.getTV(null) || angleController.atSetpoint() ? 0
                : angleController.calculate(Math.toRadians(LimelightHelpers.getTX(null)));

        drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    public void voltageDrive(Measure<Voltage> voltMeasure) {
        for (SwerveModule module : _modules) {
            module.driveByVoltage(voltMeasure.magnitude());
        }
    }

    public void periodic() {
        // Update the odometry in the periodic block
        _estimator.update(getRotation2d(), getSwerveModulePositions());

        // TODO: Move to seperate loop to not hold up main loop
        if (UPDATE_TELEMETRY)
            updateTelemetry();
        if (UPDATE_DASHBOARD)
            updateSDB();
    }

    public void disabledInit() {
        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].disable();
        }
    }

    /************************
     * SwerveModule Methods *
     ************************/

    private void calibrateSteering() {
        for (SwerveModule swerveModule : _modules) {
            swerveModule.calibrateSteering();
        }
    }

    private void setDesiredStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                DrivetrainConstants.SwerveModuleConstants.driveFreeSpeedMetersPerSecond);

        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].setDesiredState(moduleStates[i]);
        }
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[this._modules.length];

        for (int i = 0; i < swerveModulePositions.length; i++) {
            swerveModulePositions[i] = this._modules[i].getSwerveModulePosition();
        }

        return swerveModulePositions;
    }

    /***************************
     * Telemetry and Dashboard *
     ***************************/

    private void initTelemetry() {
        DataLog log = DataLogManager.getLog();
        m_poseLog = StructLogEntry.create(log, "/drivetrain/position", new Pose2dStruct());
        m_modulesCurrentStateLog = StructArrayLogEntry.create(log, "/drivetrain/modules/currentState",
                new SwerveModuleStateStruct());
        m_modulesWantedStateLog = StructArrayLogEntry.create(log, "/drivetrain/modules/wantedState",
                new SwerveModuleStateStruct());
    }

    private void updateTelemetry() {
        m_poseLog.append(getPose());

        SwerveModuleState[] currentStates = new SwerveModuleState[4];
        SwerveModuleState[] wantedStates = new SwerveModuleState[4];

        for (int i = 0; i < _modules.length; i++) {
            currentStates[i] = _modules[i].getState();
            wantedStates[i] = _modules[i].getWantedState();
        }

        m_modulesCurrentStateLog.append(currentStates);
        m_modulesWantedStateLog.append(wantedStates);
    }

    private void updateSDB() {
        for (int i = 0; i < this._modules.length; i++) {
            _modules[i].updateSDB(i);
        }

        SmartDashboard.putNumber("rotation", getRotation2d().getRadians());
    }

    /************************
     * Autonomous Interface *
     ************************/
    public Pose2d getPose() {
        return _estimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        _estimator.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        _estimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /******************************
     * Pigeon Methods *
     ******************************/

    // TODO: Verify Pigeon axis are aligned with robot in position and direction
    // - Front is shooter's direction
    // - CW Yaw is positive

    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this._pigeon.getFusedHeading());
    }

    public double getRoll() {
        return _pigeon.getRoll();
    }

    public void resetYaw() {
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
}