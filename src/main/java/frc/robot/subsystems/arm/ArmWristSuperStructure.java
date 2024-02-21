package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm.ArmState;
import frc.robot.subsystems.arm.Wrist.WristState;

public class ArmWristSuperStructure extends SubsystemBase {

    private Arm m_Arm;
    private Wrist m_Wrist;

    public ArmWristSuperStructure() {
        this.m_Arm = new Arm();
        this.m_Wrist = new Wrist(m_Arm::getPositionDeg);
    }

    private WristState convertArmToWristState(ArmState state) {
        switch (state) {
            case ALIGN:
                return WristState.TRAP;
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
            m_Arm.getSetStateCommand(targetState),
            m_Wrist.getSetStateCommand(convertArmToWristState(targetState))
        );
    }

    public Command setEncodersToCollect() {
        return new InstantCommand(() -> m_Arm.setSensorPosition(ArmState.COLLECT.armAngleDeg)).alongWith(
            new InstantCommand(() -> m_Wrist.setSensorPosition(WristState.COLLECT.angleDeg))).alongWith(
                new PrintCommand("called")
            );
    }

    public Command getArmSetState(ArmState targetState){
       return  m_Arm.getSetStateCommand(targetState);
    }

    public Command getWristStateCommand(WristState targState){
        return m_Wrist.getSetStateCommand(targState);
    }
    public void stop() {
        m_Arm.stop();
        m_Wrist.stop();
    }

    public ArmState getArmState() {
        return m_Arm.getTargetState();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && (m_Arm.isLeftLimitSwitchClosed() || m_Arm.isRightLimitSwitchClosed())) {
            m_Wrist.setSensorPosition(WristState.COLLECT.angleDeg);
        }
    }
}
