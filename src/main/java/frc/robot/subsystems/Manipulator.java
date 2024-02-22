package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.motors.DBugSparkMax;

public class Manipulator extends SubsystemBase {

    private DBugSparkMax _manipulatorMotor;

    private DigitalInput _hasNoteSwitch;

    private ManipulatorState _state;

    public static enum ManipulatorState {
        OFF(ManipulatorConstants.offPercentage),
        COLLECT(ManipulatorConstants.collectingPercentage),
        AMP(ManipulatorConstants.AMPPercentage),
        TRAP(ManipulatorConstants.TRAPPercentage),
        PRE_TRAP(ManipulatorConstants.PreTrapPercentage),
       TO_SHOOTER(ManipulatorConstants.toShooterPercentage);

        public double percentage;

        private ManipulatorState(double percentage) {
            this.percentage = percentage;
        }
    }

    public Manipulator() {
        this._manipulatorMotor = DBugSparkMax.create(ManipulatorConstants.sparkmaxCANID);
        this._manipulatorMotor.setSmartCurrentLimit(20);

        this._hasNoteSwitch = new DigitalInput(ManipulatorConstants.noteSwitchPort);

        this._state = ManipulatorState.OFF;

        // initialize values into the SDB
        SmartDashboard.putString("Manipulator State", this._state.toString());
    }

    public ManipulatorState getManipulatorState() {
        return this._state;
    }

    public boolean hasNoteSwitch() {
        return !_hasNoteSwitch.get();
    }

    private void setState(ManipulatorState state) {
        this._state = state;

        this._manipulatorMotor.set(state.percentage);

        SmartDashboard.putString("Manipulator State", this._state.toString());
    }

    public Command getSetStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(ManipulatorState.OFF);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("has note", hasNoteSwitch());
        SmartDashboard.putNumber("manipulator current current", _manipulatorMotor.getOutputCurrent());
    }
}
