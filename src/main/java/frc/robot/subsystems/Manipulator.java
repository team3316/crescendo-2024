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
    public static enum ManipulatorState {
        AMP(ManipulatorConstants.manipulatorAMPState),
        HOLD(ManipulatorConstants.manipulatorHOLDState),
        TRAP(ManipulatorConstants.manipulatorTRAPState),
        SHOOTER(ManipulatorConstants.manipulatorSHOOTERState);

        public final double percentage;
        ManipulatorState(double value) {
            this.percentage = value;
        }
    }

    private DBugSparkMax _sparkMaxLowerLeader;
    private DBugSparkMax _sparkMaxUpperFollower;
    private ManipulatorState _manipulatorState;

    public Manipulator() {
        // TODO: check what ports it should be
        this._sparkMaxLowerLeader = new DBugSparkMax(ManipulatorConstants.ManipulatorLowerSparkMaxPort);
        this._sparkMaxUpperFollower = new DBugSparkMax(ManipulatorConstants.ManipulatorUpperSparkMaxPort);

        // TODO: check if really inverted
        this._sparkMaxUpperFollower.setInverted(true);

        this._sparkMaxUpperFollower.follow(_sparkMaxLowerLeader);

        this._manipulatorState = ManipulatorState.HOLD;
    }

    public ManipulatorState getManipulatorState() {
        return this._manipulatorState;
    }

    public Command getSetStateCommand(ManipulatorState state) {
        // does not stop the manipulator processes, just start (by given state)
        return new InstantCommand(() -> setState(state), this);
    }

    private void setState(ManipulatorState state) {
        this._manipulatorState = state;

        this._sparkMaxLowerLeader.set(state.percentage);

        SmartDashboard.putString("Manipulator State:", _manipulatorState.toString());
    }

    @Override
    public void periodic() {
        //prints the position to the SmartDashboard
        SmartDashboard.putNumber("percent", _sparkMaxLowerLeader.get());
    }
}
