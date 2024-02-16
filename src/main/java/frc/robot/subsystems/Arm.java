package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

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
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Arm extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;
    private TalonFX _manipulatorJoint;

    private ArmFeedforward _armFeedforward;

    private ArmState _targetState;

    public static enum ArmState {
        COLLECT(ArmConstants.collectAngle, ArmConstants.collectManipulatorAngle),
        AMP(ArmConstants.AMPAngle, ArmConstants.AMPManipulatorAngle),
        UNDER_CHAIN(ArmConstants.underChainAngle, ArmConstants.underChainManipulatorAngle),
        TRAP(ArmConstants.TRAPAngle, ArmConstants.TRAPManipulatorAngle);

        public final double armAngleDeg;
        public final double manipulatorAngleDeg;

        private ArmState(double angleDeg, double manipulatorAngleDeg) {
            this.armAngleDeg = angleDeg;
            this.manipulatorAngleDeg = manipulatorAngleDeg;
        }
    }

    public Arm() {
        _leader = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.armKp), 
        ArmConstants.armPositionFactor, ArmConstants.armVelocityFactor, 0);
        _follower = DBugSparkMax.create(ArmConstants.followerCANID, new PIDFGains(ArmConstants.armKp), 
        ArmConstants.armPositionFactor, ArmConstants.armVelocityFactor, 0);
        _follower.follow(_leader, false); // TODO: verify inversion before testing
        _leader.setSoftLimit(SoftLimitDirection.kForward, (float)ArmState.TRAP.armAngleDeg + 2); // TODO: check which side (fwd/rev) is soft and which is hard limit
        _leader.enableSoftLimit(SoftLimitDirection.kForward, true);
        
        _manipulatorJoint = new TalonFX(ArmConstants.jointCANID);
        TalonFXConfiguration jointConfig = new TalonFXConfiguration();
        jointConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        jointConfig.Slot0.withKP(ArmConstants.manipulatorJointKp).withKV(ArmConstants.manipulatorJointKv).
        withKG(ArmConstants.armKg);
        jointConfig.Feedback.withSensorToMechanismRatio(1 / ArmConstants.manipulatorJointPositionFactor);
        _manipulatorJoint.getConfigurator().apply(jointConfig);

        _armFeedforward = new ArmFeedforward(ArmConstants.armKs, ArmConstants.armKg, ArmConstants.armKv, ArmConstants.armKa);
        
        _targetState = getInitialState();
        _leader.setPosition(_targetState.armAngleDeg);
    }

    private ArmState getInitialState() {
        if (isArmRevLimitSwitchClosed()) {
            return ArmState.COLLECT;
        }
        return ArmState.TRAP;
    }

    public ArmState getTargetState() {
        return _targetState;
    }

    // TODO: check if switches are NC or NO
    public boolean isArmRevLimitSwitchClosed() {
        return _leader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public boolean isManipulatorFwdLimitSwitchClosed() {
        return _manipulatorJoint.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean isManipulatorRevLimitSwitchClosed() {
        return _manipulatorJoint.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public double getArmPositionDeg() {
        return _leader.getPosition();
    }

    public double getArmVelocityDegPerSec() {
        return _leader.getVelocity();
    }

    private State getCurrentArmTrapezoidState() {
        return new State(getArmPositionDeg(), getArmVelocityDegPerSec());
    }

    public double getMnaipulatorJointPositionDeg() {
        return _manipulatorJoint.getPosition().getValueAsDouble();
    }

    public double getManipulatorJointVelocityDegPerSec() {
        return _manipulatorJoint.getPosition().getValueAsDouble();
    }

    private State getManipulatorJointTrapezoidState() {
        return new State(getMnaipulatorJointPositionDeg(), getManipulatorJointVelocityDegPerSec());
    }

    private void useArmState(TrapezoidProfile.State targetState) {
        double feedforward = _armFeedforward.calculate(Math.toRadians(targetState.position), Math.toRadians(targetState.velocity));
        _leader.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("target arm position (deg)", targetState.position);
        SmartDashboard.putNumber("target arm velocity (deg/sec)", targetState.velocity);
    }

    private Command getSetArmStateCommand(ArmState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.armProfileConstrains);
        Supplier<State> targetSupplier = () -> (new State(targetState.armAngleDeg, 0));
        return new TrapezoidProfileCommand(profile, this::useArmState, targetSupplier, this::getCurrentArmTrapezoidState, this);
        // no need for dynamic command as the new TrapezoidProfileCommand gets the start and goal states as Suppliers
    }

    private void useManipulatorJointState(TrapezoidProfile.State targetState) {
        _manipulatorJoint.setControl(new PositionVoltage(
            targetState.position, targetState.velocity, false, 0, 0, false, true, true));
        SmartDashboard.putNumber("target manipulator joint position (deg)", targetState.position);
        SmartDashboard.putNumber("target manipulator joint velocity (deg/sec)", targetState.velocity);
    }

    private Command getSetManipulatorJointStateCommand(ArmState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.manipulatorJointProfileConstrains);
        Supplier<State> targetSupplier = () -> (new State(targetState.manipulatorAngleDeg, 0));
        return new TrapezoidProfileCommand(profile, this::useManipulatorJointState, targetSupplier, this::getManipulatorJointTrapezoidState, this);
    }

    public Command getSetStateCommand(ArmState targetState) {
        Command toReturn = Commands.parallel(
            getSetArmStateCommand(targetState),
            getSetManipulatorJointStateCommand(targetState),
            new InstantCommand(() -> {_targetState = targetState;})
        );
        toReturn.addRequirements(this);
        return toReturn;
    }

    public void stop() {
        _leader.set(0);
        _manipulatorJoint.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putString("target arm state", _targetState.toString());
        SmartDashboard.putNumber("current arm position (deg)", getArmPositionDeg());
        SmartDashboard.putNumber("current arm velocity (deg/s)", getArmVelocityDegPerSec());
        SmartDashboard.putNumber("current manipulator joint position", getMnaipulatorJointPositionDeg());
        SmartDashboard.putNumber("current manipulator joint velocity", getManipulatorJointVelocityDegPerSec());
        SmartDashboard.putBoolean("arm rev limit", isArmRevLimitSwitchClosed());
        SmartDashboard.putBoolean("manipulator fwd limit", isManipulatorFwdLimitSwitchClosed());
        SmartDashboard.putBoolean("manipulator rev limit", isManipulatorRevLimitSwitchClosed());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            if (isArmRevLimitSwitchClosed()) {
                _leader.setPosition(ArmState.COLLECT.armAngleDeg);
            }
            if (isManipulatorRevLimitSwitchClosed()) {
                _manipulatorJoint.setPosition(ArmState.COLLECT.manipulatorAngleDeg);
            }
            else if (isManipulatorFwdLimitSwitchClosed()) {
                _manipulatorJoint.setPosition(ArmState.TRAP.manipulatorAngleDeg);
            }
        }
        updateSDB();
    }
}
