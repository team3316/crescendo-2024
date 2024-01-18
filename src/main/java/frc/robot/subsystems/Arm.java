package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
        TRAP(ArmConstants.TRAPAngle),
        CLIMB(ArmConstants.climbAngle);

        public final double angleDeg;

        private ArmState(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }

    public Arm() {
        _targetState = getInitialState();

        _leader = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, _targetState.angleDeg);
        _follower = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, _targetState.angleDeg);
        _follower.follow(_leader, true); //TODO: check if inverted

        _feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);
    }

    private ArmState getInitialState() {
        // TODO: check the switches' position and whether it NC or NO
        if (_leader.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
            return ArmState.CLIMB;
        }
        else if (_leader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
            return ArmState.COLLECT;
        }
        return ArmState.TRAP;
    }

    public ArmState getTargetState() {
        return _targetState;
    }

    private void useState(TrapezoidProfile.State targetState) {
        double feedforward = _feedforward.calculate(Math.toRadians(targetState.position), Math.toRadians(targetState.velocity));
        _leader.setReference(targetState.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("Arm position (deg)", _leader.getPosition());
    }

    private Command getSetStateCommand(ArmState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.profileConstrains);
        Supplier<State> targetSupplier = () -> (new State(targetState.angleDeg, 0));
        return new InstantCommand(() -> {_targetState = targetState;}).andThen(
            new TrapezoidProfileCommand(profile, this::useState, targetSupplier, this::getCurrentTrapezoidState, this));
        // no need for dynamic command as the new TrapezoidProfileCommand gets the start and goal states as Suppliers
    }

    public double getArmPositionDeg() {
        return _leader.getPosition();
    }

    public double getArmVelocityDeg() {
        return _leader.getVelocity();
    }

    private State getCurrentTrapezoidState() {
        return new State(getArmPositionDeg(), getArmVelocityDeg());
    }
}
