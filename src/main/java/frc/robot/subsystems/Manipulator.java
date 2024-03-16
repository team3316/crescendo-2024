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

    private static final boolean UPDATE_DASHBOARD = true;

    private DBugSparkMax _manipulatorMotor;

    private DigitalInput _lowBeamBreak;
    private DigitalInput _highBeamBreak;

    private ManipulatorState _state;

    private boolean _noteLatch = false;

    public static enum ManipulatorState {
        OFF(ManipulatorConstants.offPercentage),
        COLLECT(ManipulatorConstants.collectingPercentage),
        SLOW_COLLECT(ManipulatorConstants.slowCollectPercentage),
        AMP(ManipulatorConstants.AMPPercentage),
        TRAP(ManipulatorConstants.TRAPPercentage),
        PRE_TRAP(ManipulatorConstants.PreTrapPercentage),
        TO_SHOOTER(ManipulatorConstants.toShooterPercentage),
        EJECT(ManipulatorConstants.ejectPercentage);

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
        this._manipulatorMotor.setSmartCurrentLimit(40);

        this._lowBeamBreak = new DigitalInput(ManipulatorConstants.lowBeamBreakPort);
        this._highBeamBreak = new DigitalInput(ManipulatorConstants.highBeamBreakPort);

        this._state = ManipulatorState.OFF;

        // initialize values into the SDB
        SmartDashboard.putString("Manipulator/State", this._state.toString());
    }

    public ManipulatorState getManipulatorState() {
        return this._state;
    }

    public boolean hasNote() {
        return !_lowBeamBreak.get() || !_highBeamBreak.get();
    }

    private void setState(ManipulatorState state) {
        this._state = state;

        this._manipulatorMotor.set(state.percentage);

        SmartDashboard.putString("Manipulator/State", this._state.toString());
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
        if (!_noteLatch && hasNote()) {
            // Note just entered the manipulator, reset position to 0.
            _noteLatch = true;
            _manipulatorMotor.setPosition(0);
        } else if (_noteLatch && !hasNote()) {
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

    private void updateSDB() {
        SmartDashboard.putNumber("Manipulator/note position", getNotePosition());
        SmartDashboard.putNumber("Manipulator/current", _manipulatorMotor.getOutputCurrent());
    }
    @Override
    public void periodic() {
        resetNotePositionPeriodic();

        SmartDashboard.putBoolean("Manipulator/has note", hasNote());
        if(UPDATE_DASHBOARD) {
            updateSDB();
        }
    }
}
