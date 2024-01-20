package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.motors.DBugSparkMax;

public class Manipulator extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;
    private ManipulatorState _manipulatorState;

    public static enum ManipulatorState {
        AMP(ManipulatorConstants.manipulatorAMPState),
        OFF(ManipulatorConstants.manipulatorOFFState),
        TRAP(ManipulatorConstants.manipulatorTRAPState),
        SHOOTER(ManipulatorConstants.manipulatorSHOOTERState);

        public final double percentage;
        ManipulatorState(double percentage) {
            this.percentage = percentage;
        }
    }

    public Manipulator() {
        // TODO: check what ports it should be
        this._leader = DBugSparkMax.create(ManipulatorConstants.ManipulatorLowerSparkMaxPort);
        this._follower = DBugSparkMax.create(ManipulatorConstants.ManipulatorUpperSparkMaxPort);
        this._leader.setSmartCurrentLimit(20);
        this._follower.setSmartCurrentLimit(20);

        // TODO: check if really inverted
        this._follower.follow(_leader, true);

        this._manipulatorState = ManipulatorState.OFF;
    }

    public ManipulatorState getManipulatorState() {
        return this._manipulatorState;
    }

    private void setState(ManipulatorState state) {
        this._manipulatorState = state;

        this._leader.set(state.percentage);

        SmartDashboard.putString("Manipulator State:", _manipulatorState.toString());
        SmartDashboard.putNumber("Manipulator Percentage", state.percentage);
    }

    public Command getSetStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(ManipulatorState.OFF);
    }
}
