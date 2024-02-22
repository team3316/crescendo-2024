package frc.robot.subsystems.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;

public class ArmWristSuperStructure extends SubsystemBase {

    private final Arm m_Arm;
    private final Wrist m_Wrist;
    // TODO: verify rising or falling edge (depends on NC/NO)
    private final Debouncer _recalibrationDebouncer = new Debouncer(2);

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

    @Override
    public void periodic() {
        if (_recalibrationDebouncer.calculate(m_Arm.anyLimitSwitchClosed())) {
            setEncodersToCollect();
        }
    }
}
