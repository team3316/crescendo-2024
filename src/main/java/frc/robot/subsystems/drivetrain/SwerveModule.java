package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

/**
 * SwerveModule
 */
public class SwerveModule {

    private TalonFX _driveMotor;
    private DBugSparkMax _steerMotor;

    private CANcoder _absEncoder;

    private SwerveModuleState _targetState;

    public SwerveModule(SwerveModuleConstants constants) {
        this._absEncoder = createCANcoder(constants.canCoderId, constants.cancoderZeroAngle);

        this._driveMotor = new TalonFX(constants.idDrive);

        this._steerMotor = DBugSparkMax.create(
                constants.idSteering,
                constants.steeringGains,
                SwerveModuleConstants.steeringPositionConversionFactor,
                SwerveModuleConstants.steeringVelocityConversionFactor,
                  _absEncoder.getAbsolutePosition().getValue()*360);

        this._driveMotor.getConfigurator()
                .apply(getTalonConfig(constants.driveGains, SwerveModuleConstants.drivePositionConversionFactor));

        this._targetState = getState();
    }

    private static TalonFXConfiguration getTalonConfig(PIDFGains gains,
            double conversionFactor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.withKP(gains.kP)
                .withKI(gains.kI)
                .withKD(gains.kD)
                .withKS(gains.kF)// Feedforward
                .withKV(0.12);// voltage comp

        config.Feedback.withSensorToMechanismRatio(1 / conversionFactor);
        // config.Feedback.withRotorToSensorRatio(1 / conversionFactor);

                config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);


        return config;
    }

    private static CANcoder createCANcoder(int id, double zeroAngle) {

        CANcoderConfiguration _absencoderConfig = new CANcoderConfiguration();
        CANcoder CANcoder = new CANcoder(id);
        // Always set CANcoder relative encoder to 0 on boot

        // Configure the offset angle of the magnet
        _absencoderConfig.MagnetSensor.withMagnetOffset((360 -zeroAngle)/360);
        _absencoderConfig.MagnetSensor.AbsoluteSensorRange  = AbsoluteSensorRangeValue.Unsigned_0To1;
        CANcoder.getConfigurator().apply(_absencoderConfig, 0.3);

        return CANcoder;
    }

    public void calibrateSteering() {
        double currentAngle = this._steerMotor.getPosition();
        double angleDiff = (getAbsAngle() - currentAngle) % 360;

        double targetAngle = currentAngle + angleDiff;
        if (angleDiff <= -180)
            targetAngle += 360;

        else if (angleDiff >= 180)
            targetAngle -= 360;

        if (Math.abs(targetAngle - currentAngle) > 2) // avoid sending unnecessary CAN packets
            this._steerMotor.setPosition(targetAngle);
    }

    public void stop() {
        _driveMotor.set(0);
        _steerMotor.set(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this._driveMotor.getVelocity().getValue(),
                new Rotation2d().rotateBy(Rotation2d.fromDegrees(this._steerMotor.getPosition())));
    }

    public SwerveModuleState getTargetState() {
        return _targetState;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, this._steerMotor.getPosition());

        if (state.speedMetersPerSecond != 0) // Avoid steering in place
            this._steerMotor.setReference(state.angle.getDegrees(), ControlType.kPosition);

        if (state.speedMetersPerSecond == 0)
            this.stop();
        else
            this._driveMotor.setControl(new VelocityVoltage(
                    state.speedMetersPerSecond / SwerveModuleConstants.drivePositionConversionFactor));

        _targetState = desiredState;

    }

    public void setAngle(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        this._steerMotor.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        // desired angle diff in [-360, +360]
        double _angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;

        double targetAngle = currentAngle + _angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        // Q1 undershot. We expect a CW turn.
        if (_angleDiff <= -270)
            targetAngle += 360;

        // Q2 undershot. We expect a CCW turn to Q4 & reverse direction.
        // Q3. We expect a CW turn to Q1 & reverse direction.
        else if (-90 > _angleDiff && _angleDiff > -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        }

        // Q2. We expect a CCW turn to Q4 & reverse direction.
        // Q3 overshot. We expect a CW turn to Q1 & reverse direction.
        else if (90 < _angleDiff && _angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        }

        // Q4 overshot. We expect a CCW turn.
        else if (_angleDiff >= 270)
            targetAngle -= 360;

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    public double getAbsAngle() {
        return _absEncoder.getAbsolutePosition().getValue()*360;
    }

    public void disable() {
        this.stop();
        this._steerMotor.set(0);
    }

    public double getStatorCurrent() {
        return this._driveMotor.getStatorCurrent().getValue();
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(_driveMotor.getPosition().getValue(), Rotation2d.fromDegrees(getAbsAngle()));
    }

    public void DriveByPercent(double percent) {
        this._driveMotor.setControl(new DutyCycleOut(percent));
    }

    public double getVelocity() {
        return this._driveMotor.getVelocity().getValue();
    }
}