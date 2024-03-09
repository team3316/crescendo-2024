package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Manipulator extends SubsystemBase {

    private static final boolean UPDATE_DASHBOARD = true;

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
        this._manipulatorMotor = DBugSparkMax.create(ManipulatorConstants.sparkmaxCANID,
                ManipulatorConstants.positionGains,
                ManipulatorConstants.positionFactor, ManipulatorConstants.velocityFactor, 0);
        this._manipulatorMotor.setSmartCurrentLimit(40);

        this._manipulatorMotor.setupPIDF(ManipulatorConstants.velocityGains, 1);

        this._hasNoteSwitch = new DigitalInput(ManipulatorConstants.noteSwitchPort);

        this._state = ManipulatorState.OFF;

        // initialize values into the SDB
        SmartDashboard.putString("Manipulator/State", this._state.toString());
        SmartDashboard.putNumber("Manipulator/error", -1);

         SmartDashboard.putData("manipulatorPID", new InstantCommand(()->new InstantCommand(() -> upDatePIDF(SmartDashboard.getNumber("manipulatorKp", 0),
                SmartDashboard.getNumber("manipulatorKi", 0), SmartDashboard.getNumber("manipulatorKd", 0))).schedule()));

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

    }
    
    public void setToCollect(){
        this._manipulatorMotor.setReference(ManipulatorConstants.collectingVelocity, ControlType.kVelocity,1);
    }

    public Command getCollectCommand(){
       return new InstantCommand(()->setToCollect(),this);
    }

     public void upDatePIDF(double manipulatorKp, double manipulatorKi, double manipulatorKd) {
        this._manipulatorMotor.setupPIDF(new PIDFGains(manipulatorKp, manipulatorKi, manipulatorKd, ManipulatorConstants.velocityKf));
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

    public double getVelocity() {
        return _manipulatorMotor.getVelocity();
    }

    public Command getMoveNoteToPositionCommand(NotePosition pos) {
        return new InstantCommand(
                () -> _manipulatorMotor.setReference(pos.position, ControlType.kPosition),
                this);
    }

    private void updateSDB() {
        SmartDashboard.putString("Manipulator/State", this._state.toString());
        SmartDashboard.putNumber("Manipulator/error", getVelocity() - ManipulatorConstants.collectingPercentage);

        SmartDashboard.putNumber("Manipulator/note position", getNotePosition());
        SmartDashboard.putNumber("Manipulator/current", _manipulatorMotor.getOutputCurrent());

        SmartDashboard.putNumber("manipulatorKp", SmartDashboard.getNumber("manipulatorKp", ManipulatorConstants.velocityKp));
        SmartDashboard.putNumber("manipulatorKi", SmartDashboard.getNumber("manipulatorKi", ManipulatorConstants.velocityKi));
        SmartDashboard.putNumber("manipulatorKd", SmartDashboard.getNumber("manipulatorKd", ManipulatorConstants.velocityKd));

       
    }

    @Override
    public void periodic() {
        resetNotePositionPeriodic();

        SmartDashboard.putBoolean("Manipulator/has note", hasNoteSwitch());
        if (UPDATE_DASHBOARD) {
            updateSDB();
        }
    }
}
