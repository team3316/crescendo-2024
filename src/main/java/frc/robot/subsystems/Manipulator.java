package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.motors.DBugSparkMax;

public class Manipulator extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;
    private ManipulatorState _state;

    public static enum ManipulatorState {
        AMP(ManipulatorConstants.manipulatorAMPState),
        OFF(ManipulatorConstants.manipulatorOFFState),
        TRAP(ManipulatorConstants.manipulatorTRAPState),
        SHOOTER(ManipulatorConstants.manipulatorSHOOTERState);

        public final double percentage;
        private ManipulatorState(double percentage) {
            this.percentage = percentage;
        }
    }

    public Manipulator() {
        // TODO: check what ports it should be
        this._leader = DBugSparkMax.create(ManipulatorConstants.lowerSparkMaxPort);
        this._follower = DBugSparkMax.create(ManipulatorConstants.upperSparkMaxPort);
        this._leader.setSmartCurrentLimit(20);
        this._follower.setSmartCurrentLimit(20);

        // TODO: check if really inverted
        this._follower.follow(this._leader, true);

        setState(ManipulatorState.OFF);
    }

    public ManipulatorState getManipulatorState() {
        return this._state;
    }

    private void setState(ManipulatorState state) {
        this._state = state;

        this._leader.set(state.percentage);

        SmartDashboard.putString("Manipulator State", this._state.toString());
        SmartDashboard.putNumber("Manipulator Percentage", state.percentage);
    }

    public Command getSetStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(ManipulatorState.OFF);
    }
}
