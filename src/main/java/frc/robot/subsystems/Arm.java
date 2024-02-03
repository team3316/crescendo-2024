package frc.robot.subsystems;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Arm extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;

    private ArmFeedforward _feedforward;

    private ArmState _targetState;

    public static enum ArmState {
        COLLECT(ArmConstants.collectAngle),
        AMP(ArmConstants.AMPAngle),
        UNDER_CHAIN(ArmConstants.underChainAngle),
        TRAP(ArmConstants.TRAPAngle);

        public final double angleDeg;

        private ArmState(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }

    public Arm() {
        _leader = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower = DBugSparkMax.create(ArmConstants.followerCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower.follow(_leader, false); // TODO: verify inversion before testing
        _leader.setSoftLimit(SoftLimitDirection.kForward, (float)ArmState.TRAP.angleDeg + 2); // TODO: check which side (fwd/rev) is soft and which is hard limit
        _leader.enableSoftLimit(SoftLimitDirection.kForward, true);

        _feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);

        _targetState = getInitialState();
        _leader.setPosition(_targetState.angleDeg);
    }

    private ArmState getInitialState() {
        if (isRevLimitSwitchClosed()) {
            return ArmState.COLLECT;
        }
        return ArmState.TRAP;
    }

    public ArmState getTargetState() {
        return _targetState;
    }

    // TODO: check if switches are NC or NO
    private boolean isRevLimitSwitchClosed() {
        return _leader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
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
        double feedforward = _feedforward.calculate(Math.toRadians(targetState.position), Math.toRadians(targetState.velocity));
        _leader.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("target arm position (deg)", targetState.position);
        SmartDashboard.putNumber("target arm velocity (deg/sec)", targetState.velocity);
    }

    public Command getSetStateCommand(ArmState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.profileConstrains);
        Supplier<State> targetSupplier = () -> (new State(targetState.angleDeg, 0));
        return new InstantCommand(() -> {_targetState = targetState;}).alongWith(
            new TrapezoidProfileCommand(profile, this::useState, targetSupplier, this::getCurrentTrapezoidState, this));
        // no need for dynamic command as the new TrapezoidProfileCommand gets the start and goal states as Suppliers
    }

    public void stop() {
        _leader.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putString("target arm state", _targetState.toString());
        SmartDashboard.putNumber("current arm position (deg)", getPositionDeg());
        SmartDashboard.putNumber("current arm velocity (deg/s)", getVelocityDegPerSec());
        SmartDashboard.putBoolean("arm rev limit", isRevLimitSwitchClosed());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && isRevLimitSwitchClosed()) {
            _leader.setPosition(ArmState.COLLECT.angleDeg);
        }
        updateSDB();
    }
}
