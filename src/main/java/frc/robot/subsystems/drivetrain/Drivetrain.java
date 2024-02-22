package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;

/**
 * Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private static final boolean UPDATE_TELEMETRY = false;
    private static final boolean UPDATE_DASHBOARD = false;

    private SwerveModule[] _modules;

    private PigeonIMU _pigeon;

    private SwerveDriveOdometry _odometry;

    private StructLogEntry<Pose2d> m_poseLog;

    private static PIDController angleController;

    private static PIDController goToDirectionController;

    public static enum DIRECTIONS {
        LEFT_CLIMB(0),
        RIGHT_CLIMB(0),
        DRIVER(0),
        AMP(0);

        public final double angleDeg;

        private DIRECTIONS(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }


    public Drivetrain() {
        this._modules = new SwerveModule[] {
                new SwerveModule(DrivetrainConstants.TRModule),
                new SwerveModule(DrivetrainConstants.TLModule),
                new SwerveModule(DrivetrainConstants.BRModule),
                new SwerveModule(DrivetrainConstants.BLModule)
        };

        _pigeon = new PigeonIMU(DrivetrainConstants.pigeonId);

        _odometry = new SwerveDriveOdometry(DrivetrainConstants.kinematics, getRotation2d(),
                getSwerveModulePositions());

        initTelemetry();

        angleController = new PIDController(LimelightConstants.angleKp, 0, 0);
        angleController.setTolerance(LimelightConstants.angleTol);
        angleController.setSetpoint(0);

        goToDirectionController = new PIDController(DrivetrainConstants.goToDirectionKp, 0, 0);
        goToDirectionController.enableContinuousInput(-180, 180);

        calibrateSteering();

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

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d rightJoystick) {
        DIRECTIONS targetDirection = null;
        double rightJoystickMag = rightJoystick.getNorm();
        double rightJoystickAngleDeg = rightJoystick.getAngle().getDegrees();

        if (rightJoystickMag >= DrivetrainConstants.rightJoystickDeadband) {
            for (DIRECTIONS d : DIRECTIONS.values()) {
                if (Math.abs(d.angleDeg - rightJoystickAngleDeg) <= DrivetrainConstants.rightJoystickAngleErrorDeg) {
                    targetDirection = d;
                    break;
                }
            }
        }

        SmartDashboard.putNumber("Joystick Mag", rightJoystickMag);
        SmartDashboard.putNumber("Joystick Angle", rightJoystickAngleDeg);

        if(targetDirection!=null){
            if(goToDirectionController.getSetpoint() != targetDirection.angleDeg){
                goToDirectionController.setSetpoint(targetDirection.angleDeg);
            }

            rot = Math.toRadians(goToDirectionController.calculate(getHeading()));
        }
        drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /**
     * Drives by X and Y inputs, maintaining given target angle given from vision
     * target
     * 
     * @param xSpeed        speed in the right direction (negative is left)
     * @param ySpeed        speed in the front direction (negative is back)
     * @param angleRad      relative angle to rotate to in radians (CW is positive)
     * @param hasTarget     does vision have a target
     * @param fieldRelative treat x-y relative odometry pose
     */
    public void driveByVision(double xSpeed, double ySpeed, double angleRad, boolean hasTarget, boolean fieldRelative) {
        // If vision doesn't have a target, or we reached our setpoint, don't rotate.
        var rot = !hasTarget || angleController.atSetpoint() ? 0 : angleController.calculate(angleRad);

        drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    public void voltageDrive(Measure<Voltage> voltMeasure) {
        for (SwerveModule module : _modules) {
            module.driveByVoltage(voltMeasure.magnitude());
        }
    }

    public void periodic() {
        // Update the odometry in the periodic block
        _odometry.update(getRotation2d(), getSwerveModulePositions());

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
    }

    private void updateTelemetry() {
        m_poseLog.append(getPose());
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
        return _odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        _odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
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

    public double getHeading() {
        return this._pigeon.getFusedHeading();
    }

    public void resetYaw() {
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
}
