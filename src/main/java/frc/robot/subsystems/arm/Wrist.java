package frc.robot.subsystems.arm;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.arm.ArmWristSuperStructure.ArmWristState;

public class Wrist extends SubsystemBase {

    private static final boolean UPDATE_DASHBOARD = true;
    private static final boolean UPDATE_TELEMETRY = false;

    private TalonFX _wristMotor;

    private Supplier<Double> _armAngleDeg;
    private final PositionVoltage request;

    private ArmFeedforward _feedforward;

    private DoubleLogEntry m_VelLog;

    public Wrist(Supplier<Double> armAngleDeg) {
        _wristMotor = new TalonFX(WristConstants.wristCANID);
        _wristMotor.getConfigurator().apply(getConfigurator());
        _wristMotor.setInverted(true);

        _feedforward = new ArmFeedforward(0, WristConstants.kg, WristConstants.kv);
        _armAngleDeg = armAngleDeg;

        request = new PositionVoltage(0);

        initTelemetry();
    }

    private TalonFXConfiguration getConfigurator() {
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        wristConfig.Slot0.withKP(WristConstants.kp);
        wristConfig.Feedback.withSensorToMechanismRatio(1 / WristConstants.positionFactor);
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.withStatorCurrentLimit(20);
        currentConfigs.withStatorCurrentLimitEnable(true);
        wristConfig.withCurrentLimits(currentConfigs);
        SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs();
        limitConfigs.withForwardSoftLimitThreshold(
                ArmWristState.AMP.wristAngleDeg + WristConstants.softLimitExtraAngle);
        limitConfigs.withForwardSoftLimitEnable(true);
        limitConfigs.withReverseSoftLimitThreshold(ArmWristState.COLLECT.wristAngleDeg
                - WristConstants.softLimitExtraAngle);
        limitConfigs.withReverseSoftLimitEnable(true);
        wristConfig.withSoftwareLimitSwitch(limitConfigs);
        return wristConfig;
    }

    private void initTelemetry() {
        DataLog log = DataLogManager.getLog();
        m_VelLog = new DoubleLogEntry(log, "/wrist/velocity");
    }

    private void updateTelemetry() {
        m_VelLog.append(getVelocityDegPerSec());
    }

    private double getPositionDeg() {
        return _wristMotor.getPosition().getValueAsDouble();
    }

    private double getAbsolutePositionDeg() {
        return getPositionDeg() + _armAngleDeg.get();
    }

    private double getVelocityDegPerSec() {
        return _wristMotor.getVelocity().getValueAsDouble();
    }

    private State getTrapezoidState() {
        return new State(getPositionDeg(), getVelocityDegPerSec());
    }

    public void setSensorPosition(double angleDeg) {
        _wristMotor.setPosition(angleDeg);
    }

    private void useState(TrapezoidProfile.State targetState) {
        double feedforward = _feedforward.calculate(Math.toRadians(targetState.position + _armAngleDeg.get()),
                Math.toRadians(targetState.velocity));

        _wristMotor.setControl(request.withPosition(targetState.position)
                .withVelocity(targetState.velocity)
                .withFeedForward(feedforward));

        SmartDashboard.putNumber("Wrist/target wrist position (deg)", targetState.position);
        SmartDashboard.putNumber("Wrist/target wrist velocity (deg|sec)", targetState.velocity);
    }

    private Command generateSetStateCommand(ArmWristState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(WristConstants.profileConstrains,
                new State(targetState.wristAngleDeg, 0), getTrapezoidState());
        return new InstantCommand(this::stop)
                .andThen(
                        new TrapezoidProfileCommand(profile, this::useState, this))
                .andThen(getHoldCommand(targetState));
    }

    private Command getHoldCommand(ArmWristState targetState) {
        return new ConditionalCommand(new InstantCommand(() -> _wristMotor.set(-0.05)), new InstantCommand(),
                () -> targetState == ArmWristState.COLLECT);
    }

    public Command getSetStateCommand(ArmWristState targetState) {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(this);
        return Commands.defer(() -> generateSetStateCommand(targetState), requirements);
    }

    public void stop() {
        _wristMotor.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Wrist/wrist position", getPositionDeg());
        SmartDashboard.putNumber("Wrist/wrist velocity", getVelocityDegPerSec());
        SmartDashboard.putNumber("Wrist/wrist absolute position", getAbsolutePositionDeg());
    }

    public void setBrakeMode(boolean shouldBreak) {
        NeutralModeValue mode = shouldBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        _wristMotor.setNeutralMode(mode);
    }

    @Override
    public void periodic() {
        if (UPDATE_DASHBOARD) {
            updateSDB();
        }
        if (UPDATE_TELEMETRY) {
            updateTelemetry();
        }
    }

}
