package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.SynchronousInterrupt.WaitResult;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm.ArmState;
import frc.robot.subsystems.arm.Wrist.WristState;

public class ArmWristSuperStructure extends SubsystemBase {

    private Arm m_Arm;
    private Wrist m_Wrist;

    public ArmWristSuperStructure(Arm arm, Wrist wrist) {
        this.m_Arm = arm;
        this.m_Wrist = m_Wrist;
    }

    private WristState convertArmToWristState(ArmState state) {
        switch (state) {
            case COLLECT:
                return WristState.COLLECT;
            case AMP:
                return WristState.AMP;
            case TRAP:
                return WristState.TRAP;
            case UNDER_CHAIN:
                return WristState.UNDER_CHAIN;
            default:
                return WristState.COLLECT;
        }
    }

    public Command getSetStateCommand(ArmState targetState) {
        return Commands.sequence(
            m_Wrist.getSetStateCommand(WristState.COLLECT),
            getSetStateCommand(targetState),
            m_Wrist.getSetStateCommand(convertArmToWristState(targetState))
        );
    }

    public void stop() {
        m_Arm.stop();
        m_Wrist.stop();
    }

    @Override
    public void periodic() {
        m_Wrist.setCurrentArmState(m_Arm.getTargetState());
    }
}
