package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.arm.Arm.ArmState;

public class Wrist extends SubsystemBase {

    private TalonFX _wristMotor;

    private WristState _targetState;
    private ArmState _currentArmState;

    public static enum WristState {
        COLLECT(WristConstants.collectAngle),
        AMP(WristConstants.AMPAngle),
        TRAP(WristConstants.TRAPAngle),
        UNDER_CHAIN(WristConstants.underChainAngle);

        public final double angleDeg;

        private WristState(double angleDeg) {
            this.angleDeg = angleDeg;
        }

        public double getAngleToGroundDeg(double armAngleDeg) {
            return this.angleDeg + armAngleDeg;
        }
    }

    public Wrist() {
        _wristMotor = new TalonFX(WristConstants.wristCANID);
        _wristMotor.getConfigurator().apply(getConfigurator());
    }

    private TalonFXConfiguration getConfigurator() {
        TalonFXConfiguration jointConfig = new TalonFXConfiguration();
        jointConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        jointConfig.Slot0.withKP(WristConstants.kp).withKV(WristConstants.kv).withKG(ArmConstants.kg);
        jointConfig.Feedback.withSensorToMechanismRatio(1 / WristConstants.positionFactor);
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.withStatorCurrentLimit(20);
        currentConfigs.withStatorCurrentLimitEnable(true);
        jointConfig.withCurrentLimits(currentConfigs);
        return jointConfig;
    }

    public WristState getWristState() {
        return _targetState;
    }

    public double getPositionDeg() {
        return _wristMotor.getPosition().getValueAsDouble();
    }

    public double getVelocityDegPerSec() {
        return _wristMotor.getVelocity().getValueAsDouble();
    }

    private State getTrapezoidState() {
        return new State(getPositionDeg(), getVelocityDegPerSec());
    }

    public boolean isFwdLimitSwitchClosed() {
        return _wristMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean isRevLimitSwitchClosed() {
        return _wristMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public void setSensorPosition(double angleDeg) {
        _wristMotor.setPosition(angleDeg);
    }

    private void useState(TrapezoidProfile.State targetState) {
        _wristMotor.setControl(new PositionVoltage(
                targetState.position, targetState.velocity, false, 0, 0, false, true, true));
        SmartDashboard.putNumber("target wrist position (deg)", targetState.position);
        SmartDashboard.putNumber("target wrist velocity (deg/sec)", targetState.velocity);
    }

    public Command getSetStateCommand(WristState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(WristConstants.profileConstrains);
        Supplier<State> targetSupplier = () -> (new State(targetState.getAngleToGroundDeg(_currentArmState.armAngleDeg),
                0));
        return (new InstantCommand(() -> {
            _wristMotor.setPosition(targetState.getAngleToGroundDeg(_currentArmState.armAngleDeg));
        }).alongWith(
            new InstantCommand(() -> {_targetState = targetState;})
        )).andThen(
                new TrapezoidProfileCommand(profile, this::useState, targetSupplier, this::getTrapezoidState, this));
    }

    public void stop() {
        _wristMotor.set(0);
    }

    public void setCurrentArmState(ArmState currentState) {
        this._currentArmState = currentState;
        TalonFXConfiguration config = getConfigurator();
        SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs();
        limitConfigs.withForwardSoftLimitThreshold(
                WristState.TRAP.getAngleToGroundDeg(_currentArmState.armAngleDeg) + WristConstants.softLimitExtraAngle);
        limitConfigs.withForwardSoftLimitEnable(true);
        limitConfigs.withReverseSoftLimitThreshold(WristState.COLLECT.getAngleToGroundDeg(_currentArmState.armAngleDeg)
                - WristConstants.softLimitExtraAngle);
        limitConfigs.withReverseSoftLimitEnable(true);
        config.withSoftwareLimitSwitch(limitConfigs);
        _wristMotor.getConfigurator().apply(config);
    }

    private void updateSDB() {
        SmartDashboard.putBoolean("wrist fwd limit", isFwdLimitSwitchClosed());
        SmartDashboard.putBoolean("wrist rev limit", isRevLimitSwitchClosed());
        SmartDashboard.putNumber("current wrist position", getPositionDeg());
        SmartDashboard.putNumber("current wrist velocity", getVelocityDegPerSec());
        SmartDashboard.putString("target wrist state", _targetState.toString());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            if (isFwdLimitSwitchClosed()) {
                setSensorPosition((WristState.TRAP.getAngleToGroundDeg(_currentArmState.armAngleDeg)));
            } else if (isRevLimitSwitchClosed()) {
                setSensorPosition(WristState.COLLECT.getAngleToGroundDeg(_currentArmState.armAngleDeg));
            }
        }
        updateSDB();
    }

}
