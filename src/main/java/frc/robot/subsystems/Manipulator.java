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

    private DBugSparkMax _sparkMaxLowerLeader;
    private DBugSparkMax _sparkMaxUpperFollower;
    private ManipulatorState _manipulatorState;

    public static enum ManipulatorState {
        AMP(ManipulatorConstants.manipulatorAMPState),
        HOLD(ManipulatorConstants.manipulatorHOLDState),
        TRAP(ManipulatorConstants.manipulatorTRAPState),
        SHOOTER(ManipulatorConstants.manipulatorSHOOTERState);

        public final double percentage;
        ManipulatorState(double percentage) {
            this.percentage = percentage;
        }
    }

    public Manipulator() {
        // TODO: check what ports it should be
        this._sparkMaxLowerLeader = DBugSparkMax.create(ManipulatorConstants.ManipulatorLowerSparkMaxPort);
        this._sparkMaxUpperFollower = DBugSparkMax.create(ManipulatorConstants.ManipulatorUpperSparkMaxPort);

        // TODO: check if really inverted
        this._sparkMaxUpperFollower.follow(_sparkMaxLowerLeader, true);

        this._manipulatorState = ManipulatorState.HOLD;
    }

    public ManipulatorState getManipulatorState() {
        return this._manipulatorState;
    }

    private void setState(ManipulatorState state) {
        this._manipulatorState = state;

        this._sparkMaxLowerLeader.set(state.percentage);

        SmartDashboard.putString("Manipulator State:", _manipulatorState.toString());
        SmartDashboard.putNumber("Manipulator Percentage", state.percentage);
    }

    public Command getSetStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> setState(state), this);
    }
}
