package frc.robot.subsystems.arm;

import java.util.HashSet;
import java.util.Set;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Arm extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;

    private DigitalInput _leftSwitch;
    private DigitalInput _rightSwitch;

    private ArmFeedforward _armFeedforward;

    private ArmState _targetState;

    public static enum ArmState {
        COLLECT(ArmConstants.collectAngle),
        AMP(ArmConstants.AMPAngle),
        UNDER_CHAIN(ArmConstants.underChainAngle),
        ALIGN(ArmConstants.ALIGNAngle),
        TRAP(ArmConstants.TRAPAngle);

        public final double armAngleDeg;

        private ArmState(double angleDeg) {
            this.armAngleDeg = angleDeg;
        }
    }

    public Arm() {
        _leader = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower = DBugSparkMax.create(ArmConstants.followerCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower.follow(_leader, true); // TODO: verify inversion before testing
        _leader.setSoftLimit(SoftLimitDirection.kForward, (float)ArmState.TRAP.armAngleDeg + ArmConstants.softLimitExtraAngle); // TODO: check which side (fwd/rev) is soft and which is hard limit
        _leader.enableSoftLimit(SoftLimitDirection.kForward, true);
        
        _armFeedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);
        
        _leftSwitch = new DigitalInput(ArmConstants.leftSwitchPort);
        _rightSwitch = new DigitalInput(ArmConstants.rightSwitchPort);

        _targetState = getInitialState();
        _leader.setPosition(_targetState.armAngleDeg);
    }

    private ArmState getInitialState() {
        if (isLeftLimitSwitchClosed()) {
            return ArmState.COLLECT;
        }
        return ArmState.TRAP;
    }

    public ArmState getTargetState() {
        return _targetState;
    }

    public void setSensorPosition(double position) {
        _leader.setPosition(position);
    }

    // TODO: check if switches are NC or NO
    public boolean isLeftLimitSwitchClosed() {
        return _leftSwitch.get();
    }

    public boolean isRightLimitSwitchClosed() {
        return _rightSwitch.get();
    }

    public double getPositionDeg() {
        return _leader.getPosition();
    }

    public double getVelocityDegPerSec() {
        return _leader.getVelocity();
    }

    private State getCurrentTrapezoidState() {
        return new State(getPositionDeg(), getVelocityDegPerSec());
    }

    private void useState(TrapezoidProfile.State targetState) {
        double feedforward = _armFeedforward.calculate(Math.toRadians(targetState.position), Math.toRadians(targetState.velocity));
        _leader.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("target arm position (deg)", targetState.position);
        SmartDashboard.putNumber("target arm velocity (deg/sec)", targetState.velocity);
    }

    private Command generateSetStateCommand(ArmState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.profileConstrains, new State(targetState.armAngleDeg, 0), getCurrentTrapezoidState());
        
        return new InstantCommand(this::stop).andThen(new TrapezoidProfileCommand(profile, this::useState, this)).alongWith(
            new InstantCommand(() -> {_targetState = targetState;})).andThen(
                Commands.either(
                    Commands.runOnce(() -> {_leader.set(-0.05);}, this),
                    Commands.none(),
                    () -> targetState == ArmState.COLLECT
                    ));
        // no need for dynamic command as the new TrapezoidProfileCommand gets the start and goal states as Suppliers
    }

    public Command getSetStateCommand(ArmState targetState) {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(this);
        return Commands.defer(() -> generateSetStateCommand(targetState), requirements);
    }

    public void stop() {
        _leader.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putString("target arm state", _targetState.toString());
        SmartDashboard.putNumber("current arm position (deg)", getPositionDeg());
        SmartDashboard.putNumber("current arm velocity (deg/s)", getVelocityDegPerSec());
        SmartDashboard.putBoolean("arm left limit", isLeftLimitSwitchClosed());
        SmartDashboard.putBoolean("arm right limit", isRightLimitSwitchClosed());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && (isLeftLimitSwitchClosed() || isRightLimitSwitchClosed())) {
            _leader.setPosition(ArmState.COLLECT.armAngleDeg);
        }
        updateSDB();
        // if (DriverStation.isEnabled()) {
        //     _leader.setVoltage(SmartDashboard.getNumber("kg voltage", 0) * Math.cos(Math.toRadians(getPositionDeg())) + SmartDashboard.getNumber("kv voltage", 0));
        // }
        // if (DriverStation.isEnabled()) {
        //     _leader.setVoltage(SmartDashboard.getNumber("arm voltage", 0));
        // }
        SmartDashboard.putNumber("kg voltage", SmartDashboard.getNumber("kg voltage", 0));
        SmartDashboard.putNumber("ks voltage", SmartDashboard.getNumber("ks voltage", 0));
        SmartDashboard.putNumber("kv voltage", SmartDashboard.getNumber("kv voltage", 0));
    }
}
