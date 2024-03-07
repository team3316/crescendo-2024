package frc.robot.subsystems.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;
import frc.robot.utils.LatchedBoolean;

public class ArmWristSuperStructure extends SubsystemBase {

    private final Arm m_Arm;
    private final Wrist m_Wrist;
    // TODO: verify rising or falling edge (depends on NC/NO)
    private final Debouncer _recalibrationDebouncer = new Debouncer(2);
    private DigitalInput _coastSwitch = new DigitalInput(ArmConstants.coastSwitchPort);
    private LatchedBoolean _shouldChangeMode = new LatchedBoolean();
    private boolean _isBreakMode = true;

    public ArmWristSuperStructure() {
        this.m_Arm = new Arm();
        this.m_Wrist = new Wrist(m_Arm::getPositionDeg);
    }

    public static enum ArmWristState {
        COLLECT(ArmConstants.collectAngle, WristConstants.collectAngle),
        AMP(ArmConstants.AMPAngle, WristConstants.AMPAngle),
        UNDER_CHAIN(ArmConstants.underChainAngle, WristConstants.underChainAngle),
        ALIGN(ArmConstants.ALIGNAngle, WristConstants.TRAPAngle),
        TRAP(ArmConstants.TRAPAngle, WristConstants.TRAPAngle);

        public final double armAngleDeg;
        public final double wristAngleDeg;

        private ArmWristState(double armAngleDeg, double wristAngleDeg) {
            this.armAngleDeg = armAngleDeg;
            this.wristAngleDeg = wristAngleDeg;
        }
    }

    public Command getSetStateCommand(ArmWristState targetState) {
        return Commands.sequence(
                m_Wrist.getSetStateCommand(ArmWristState.COLLECT),
                m_Arm.getSetStateCommand(targetState),
                m_Wrist.getSetStateCommand(targetState));
    }

    private void setEncodersToCollect() {
        m_Arm.setSensorPosition(ArmWristState.COLLECT.armAngleDeg);
        m_Wrist.setSensorPosition(ArmWristState.COLLECT.wristAngleDeg);
    }

    public Command getSetEncodersToCollectCommand() {
        return new InstantCommand(() -> setEncodersToCollect());
    }

    public void stop() {
        m_Arm.stop();
        m_Wrist.stop();
    }

    private void setBreakMode(boolean shouldBreak) {
        m_Arm.setBreakMode(shouldBreak);
        m_Wrist.setBreakMode(shouldBreak);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // only update once per button press - software latch
            if (_shouldChangeMode.update(!_coastSwitch.get())) {
                // flip the currently latched state
                _isBreakMode = !_isBreakMode;
                setBreakMode(_isBreakMode);
            }
        } else {
            // set break mode once if it was false
            if (!_isBreakMode) {
                    _isBreakMode = true;
                    setBreakMode(_isBreakMode);
            }
        }

        if (_recalibrationDebouncer.calculate(m_Arm.anyLimitSwitchClosed())) {
            setEncodersToCollect();
        }
    }
}
