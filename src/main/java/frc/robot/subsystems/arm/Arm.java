package frc.robot.subsystems.arm;

import java.util.HashSet;
import java.util.Set;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import frc.robot.subsystems.arm.ArmWristSuperStructure.ArmWristState;
import frc.robot.utils.LatchedBoolean;

public class Arm extends SubsystemBase {

    private static final boolean UPDATE_DASHBOARD = true;
    private static final boolean UPDATE_TELEMETRY = true;

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;

    private DigitalInput _leftSwitch;

    private ArmFeedforward _armFeedforward;

    private DoubleLogEntry m_velLog;

    private final Debouncer _recalibrationDebouncer = new Debouncer(2);
    private LatchedBoolean _limitLatchedBoolean;

    public Arm() {
        _leader = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp),
                ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower = DBugSparkMax.create(ArmConstants.followerCANID, new PIDFGains(ArmConstants.kp),
                ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower.follow(_leader, true);
        _leader.setSoftLimit(SoftLimitDirection.kForward,
                (float) ArmWristState.PRE_CLIB.armAngleDeg + ArmConstants.softLimitExtraAngle);
        _leader.enableSoftLimit(SoftLimitDirection.kForward, true);

        _armFeedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);

        _leftSwitch = new DigitalInput(ArmConstants.leftSwitchPort);

        _leader.setPosition(getInitialState().armAngleDeg);

        initTelemetry();

        _limitLatchedBoolean = new LatchedBoolean();
    }

    private void initTelemetry() {
        DataLog log = DataLogManager.getLog();
        m_velLog = new DoubleLogEntry(log, "/arm/velocity");
    }

    private void updateTelemetry() {
        m_velLog.append(getVelocityDegPerSec());
    }

    private ArmWristState getInitialState() {
        if (anyLimitSwitchClosed()) {
            return ArmWristState.COLLECT;
        }
        return ArmWristState.PRE_CLIB;
    }

    public void setSensorPosition(double position) {
        _leader.setPosition(position);
    }

    // TODO: check if switches are NC or NO
    public boolean anyLimitSwitchClosed() {
        return !_leftSwitch.get();
    }

    public double getPositionDeg() {
        return _leader.getPosition();
    }

    private double getVelocityDegPerSec() {
        return _leader.getVelocity();
    }

    private State getCurrentTrapezoidState() {
        return new State(getPositionDeg(), getVelocityDegPerSec());
    }

    private void useState(TrapezoidProfile.State targetState) {
        double feedforward = _armFeedforward.calculate(Math.toRadians(targetState.position),
                Math.toRadians(targetState.velocity));
        _leader.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("Arm/target arm position (deg)", targetState.position);
        SmartDashboard.putNumber("Arm/target arm velocity (deg/sec)", targetState.velocity);
    }

    private Command generateSetStateCommand(ArmWristState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.profileConstrains,
                new State(targetState.armAngleDeg, 0), getCurrentTrapezoidState());

        return new InstantCommand(this::stop).andThen(new TrapezoidProfileCommand(profile, this::useState, this))
                .andThen(getHoldCommand(targetState));
    }

    private Command getHoldCommand(ArmWristState targetState) {
        return new ConditionalCommand(new InstantCommand(() -> _leader.set(-0.03)), new InstantCommand(),
                () -> targetState == ArmWristState.COLLECT);
    }

    public Command getSetStateCommand(ArmWristState targetState) {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(this);
        return Commands.defer(() -> generateSetStateCommand(targetState), requirements);
    }

    private void climb() {
        if (getPositionDeg() >= ArmConstants.climbPosition) {
            _leader.set(ArmConstants.climbPercentage);
        }
        else {
            _leader.set(0);
        }
    }

    public Command getClimbCommand() {
        return new FunctionalCommand(() -> {}, this::climb, (interrupted) -> stop(), () -> false, this);
    }

    public void stop() {
        _leader.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Arm/arm position (deg)", getPositionDeg());
        SmartDashboard.putNumber("Arm/arm velocity (deg/s)", getVelocityDegPerSec());
        SmartDashboard.putBoolean("Arm/arm limit", anyLimitSwitchClosed());
        SmartDashboard.putNumber("Arm/leader current", _leader.getOutputCurrent());
        SmartDashboard.putNumber("Arm/follower current", _follower.getOutputCurrent());
    }

    public void setBrakeMode(boolean shouldBrake) {
        IdleMode mode = shouldBrake ? IdleMode.kBrake : IdleMode.kCoast;
        _leader.setIdleMode(mode);
        _follower.setIdleMode(mode);
    }

    @Override
    public void periodic() {
        if (UPDATE_DASHBOARD) {
            updateSDB();
        }
        if (UPDATE_TELEMETRY) {
            updateTelemetry();
        }

        if (_limitLatchedBoolean.update(_recalibrationDebouncer.calculate(anyLimitSwitchClosed()))) {
            setSensorPosition(ArmWristState.COLLECT.armAngleDeg);
        }
    }
}
