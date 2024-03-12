package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;

public class ArmWristSuperStructure extends SubsystemBase {

    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private DigitalInput _coastSwitch = new DigitalInput(ArmConstants.coastSwitchPort);
    private Trigger _toggleNutoralMode;

    public ArmWristSuperStructure() {
        this.m_Arm = new Arm();
        this.m_Wrist = new Wrist(m_Arm::getPositionDeg);
        this._toggleNutoralMode = new Trigger(()->DriverStation.isDisabled()&&_coastSwitch.get()).debounce(1);

        _toggleNutoralMode.toggleOnTrue(Commands.startEnd(()->setBreakMode(false),()->setBreakMode(true)).until(DriverStation::isEnabled).ignoringDisable(true));
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
        if (targetState == ArmWristState.AMP) {
            return m_Arm.getSetStateCommand(targetState)
                    .alongWith(Commands.waitUntil(() -> m_Arm.getPositionDeg() >= ArmConstants.wristMovementAngle)
                            .andThen(m_Wrist.getSetStateCommand(targetState)));
        } else {
            return Commands.sequence(
                    m_Wrist.getSetStateCommand(ArmWristState.COLLECT),
                    m_Arm.getSetStateCommand(targetState),
                    m_Wrist.getSetStateCommand(targetState),
                    m_Wrist.getHoldCommand(ArmWristState.COLLECT));
        }
    }

    public void stop() {
        m_Arm.stop();
        m_Wrist.stop();
    }

    private void setBreakMode(boolean shouldBreak) {
        m_Arm.setBreakMode(shouldBreak);
        m_Wrist.setBreakMode(shouldBreak);
    }
}
