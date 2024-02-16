package frc.robot.subsystems;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;

public class Wrist extends SubsystemBase{
    
    private TalonFX _wristMotor;

    private WristState _targertState;

    public static enum WristState {
        COLLECT(WristConstants.collectAngle),
        AMP(WristConstants.AMPAngle),
        TRAP(WristConstants.TRAPAngle),
        UNDER_CHAIN(WristConstants.underChainAngle);

        public final double angleDeg;

        private WristState(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }

    public Wrist() {
        _wristMotor = new TalonFX(WristConstants.wristCANID);
        TalonFXConfiguration jointConfig = new TalonFXConfiguration();
        jointConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        jointConfig.Slot0.withKP(WristConstants.kp).withKV(WristConstants.kv).
        withKG(ArmConstants.kg);
        jointConfig.Feedback.withSensorToMechanismRatio(1 / WristConstants.positionFactor);
        _wristMotor.getConfigurator().apply(jointConfig);

    }

    public WristState getWristState() {
        return _targertState;
    }

    public double getPositionDeg() {
        return _wristMotor.getPosition().getValueAsDouble();
    }

    public double getVelocityDegPerSec() {
        return _wristMotor.getPosition().getValueAsDouble();
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
        SmartDashboard.putNumber("target manipulator joint position (deg)", targetState.position);
        SmartDashboard.putNumber("target manipulator joint velocity (deg/sec)", targetState.velocity);
    }

    public Command getSetStateCommand(WristState targetState) {
        TrapezoidProfile profile = new TrapezoidProfile(WristConstants.profileConstrains);
        Supplier<State> targetSupplier = () -> (new State(targetState.angleDeg, 0));
        return new TrapezoidProfileCommand(profile, this::useState, targetSupplier, this::getTrapezoidState, this);
    }

    public void stop() {
        _wristMotor.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putBoolean("manipulator fwd limit", isFwdLimitSwitchClosed());
        SmartDashboard.putBoolean("manipulator rev limit", isRevLimitSwitchClosed());
        SmartDashboard.putNumber("current manipulator joint position", getPositionDeg());
        SmartDashboard.putNumber("current manipulator joint velocity", getVelocityDegPerSec());
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            if (isRevLimitSwitchClosed()) {
                _wristMotor.setPosition(WristState.COLLECT.angleDeg);
            }
            else if (isFwdLimitSwitchClosed()) {
                _wristMotor.setPosition(WristState.TRAP.angleDeg);
            }
        }
        updateSDB();
    }
    
}
