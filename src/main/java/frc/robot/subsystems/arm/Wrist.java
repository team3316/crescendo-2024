package frc.robot.subsystems.arm;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.arm.Arm.ArmState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Wrist extends SubsystemBase {

    private TalonFX _wristMotor;

    private WristState _targetState;
    private Supplier<Double> _armAngleDeg;

    private ArmFeedforward _feedforward;

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

    public Wrist(Supplier<Double> armAngleDeg) {
        _wristMotor = new TalonFX(WristConstants.wristCANID);
        _wristMotor.getConfigurator().apply(getConfigurator());
        _wristMotor.setInverted(true);

        _feedforward = new ArmFeedforward(0, WristConstants.kg, WristConstants.kv);
        _armAngleDeg = armAngleDeg;
        
        _targetState = WristState.COLLECT;
    }

    private TalonFXConfiguration getConfigurator() {
        TalonFXConfiguration jointConfig = new TalonFXConfiguration();
        jointConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        jointConfig.Slot0.withKP(WristConstants.kp);
        jointConfig.Feedback.withSensorToMechanismRatio(1 / WristConstants.positionFactor);
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.withStatorCurrentLimit(20);
        currentConfigs.withStatorCurrentLimitEnable(true);
        jointConfig.withCurrentLimits(currentConfigs);
        SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs();
        limitConfigs.withForwardSoftLimitThreshold(
                WristState.TRAP.angleDeg + WristConstants.softLimitExtraAngle);
        limitConfigs.withForwardSoftLimitEnable(true);
        limitConfigs.withReverseSoftLimitThreshold(WristState.COLLECT.angleDeg
                        - WristConstants.softLimitExtraAngle);
        limitConfigs.withReverseSoftLimitEnable(true);
        jointConfig.withSoftwareLimitSwitch(limitConfigs);
        return jointConfig;
    }

    public WristState getWristState() {
        return _targetState;
    }

    public double getPositionDeg() {
        return _wristMotor.getPosition().getValueAsDouble();
    }

    public double getAbsolutePositionDeg() {
        return getPositionDeg() + _armAngleDeg.get();
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
        double gravityFeedforward = _feedforward.calculate(Math.toRadians(targetState.position + _armAngleDeg.get()), Math.toRadians(targetState.velocity));
        _wristMotor.setControl(new PositionVoltage(
                targetState.position, targetState.velocity, false, gravityFeedforward, 0, false, true, true));
        SmartDashboard.putNumber("target wrist position (deg)", targetState.position);
        SmartDashboard.putNumber("target wrist velocity (deg/sec)", targetState.velocity);
    }

    private Command generateSetStateCommand(WristState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(WristConstants.profileConstrains, new State(targetState.angleDeg, 0), getTrapezoidState());
        return 
            new InstantCommand(() -> {_targetState = targetState;}).andThen(
                new InstantCommand(this::stop)
            )
        .andThen(
                new TrapezoidProfileCommand(profile, this::useState, this)).andThen(
                    Commands.either(
                    Commands.runOnce(() -> {_wristMotor.set(-0.03);}, this),
                    Commands.none(),
                    () -> targetState == WristState.COLLECT
                    )
                );
    }

    public Command getSetStateCommand(WristState targetState) {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(this);
        return Commands.defer(() -> generateSetStateCommand(targetState), requirements);
    }

    public void stop() {
        _wristMotor.set(0);
    }

    private void updateSDB() {
                SmartDashboard.putNumber("abs pos", this.getPositionDeg() + _armAngleDeg.get());

        SmartDashboard.putBoolean("wrist fwd limit", isFwdLimitSwitchClosed());
        SmartDashboard.putBoolean("wrist rev limit", isRevLimitSwitchClosed());
        SmartDashboard.putNumber("current wrist position", getPositionDeg());
        SmartDashboard.putNumber("current wrist velocity", getVelocityDegPerSec());
        SmartDashboard.putString("target wrist state", _targetState.toString());
        SmartDashboard.putNumber("wrist absolute position", getAbsolutePositionDeg());
        SmartDashboard.putNumber("wrist applied voltage", _wristMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            if (isFwdLimitSwitchClosed()) {
                setSensorPosition((WristState.TRAP.angleDeg));
            } else if (isRevLimitSwitchClosed()) {
                setSensorPosition(WristState.COLLECT.angleDeg);
            }
        }
        /*if (DriverStation.isEnabled()) {
            _wristMotor.setVoltage(SmartDashboard.getNumber("wrist kg", 0) * Math.cos(Math.toRadians(getAbsolutePositionDeg())));
        }*/
        SmartDashboard.putNumber("wrist kg", SmartDashboard.getNumber("wrist kg", 0));
        updateSDB();
    }

}
