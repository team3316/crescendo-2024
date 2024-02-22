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

    private DBugSparkMax _manipulatorMotor;

    private DigitalInput _hasNoteSwitch;

    private ManipulatorState _state;

    private boolean _noteLatch = false;

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

    public static enum NotePosition {
        // Not sure if EXTRACT is needed. Omit if unnecessary.
        EXTRACT(ManipulatorConstants.NotePosition.extract),
        AMP(ManipulatorConstants.NotePosition.amp),
        TRAP(ManipulatorConstants.NotePosition.trap);

        public double position;

        private NotePosition(double position) {
            this.position = position;
        }

    }

    public Manipulator() {
        this._manipulatorMotor = DBugSparkMax.create(ManipulatorConstants.sparkmaxCANID, ManipulatorConstants.gains,
                ManipulatorConstants.positionFactor, ManipulatorConstants.velocityFactor, 0);
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

    /******************************
     * Note Position Manipulation *
     ******************************/
    private void resetNotePositionPeriodic() {
        if (!_noteLatch && hasNoteSwitch()) {
            // Note just entered the manipulator, reset position to 0.
            _noteLatch = true;
            _manipulatorMotor.setPosition(0);
        } else if (_noteLatch && !hasNoteSwitch()) {
            // Note just left the manipulator, reset latch.
            _noteLatch = false;
        }
    }

    private double getNotePosition() {
        return _manipulatorMotor.getPosition();
    }

    public Command getMoveNoteToPositionCommand(NotePosition pos) {
        return new InstantCommand(
                () -> _manipulatorMotor.setReference(pos.position, ControlType.kPosition),
                this);
    }

    @Override
    public void periodic() {
        resetNotePositionPeriodic();

        SmartDashboard.putBoolean("has note", hasNoteSwitch());
        if (hasNoteSwitch())
            SmartDashboard.putNumber("manipulator note position", getNotePosition());

        SmartDashboard.putNumber("manipulator current current", _manipulatorMotor.getOutputCurrent());
    }
}
