package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.motors.DBugSparkMax;

public class Intake extends SubsystemBase {

    private static final boolean UPDATE_DASHBOARD = true;

    private DBugSparkMax _intakeMotor;
    private DBugSparkMax _intakeRoller;

    private DigitalInput _hasNoteSwitch;

    private IntakeState _state;

    public static enum IntakeState {
        COLLECTING(IntakeConstants.collectingPercentage),
        EJECT(IntakeConstants.ejectPercentage),
        DISABLED(IntakeConstants.disabledPrecent);

        public final double percentage;

        private IntakeState(double precentege) {
            this.percentage = precentege;
        }
    }

    public Intake() {
        this._hasNoteSwitch = new DigitalInput(IntakeConstants.sensorID);

        _intakeMotor = DBugSparkMax.create(IntakeConstants.intakeMotorID);
        _intakeMotor.setSmartCurrentLimit(15);
        _intakeRoller = DBugSparkMax.create(IntakeConstants.rollerID);
        _intakeRoller.setSmartCurrentLimit(15);

        this._state = IntakeState.DISABLED;
    }

    public boolean isNoteInIntake() {
        return !_hasNoteSwitch.get();
    }

    public IntakeState getState() {
        return this._state;
    }

    private void setState(IntakeState state) {
        this._state = state;

        _intakeMotor.set(state.percentage);
        _intakeRoller.set(state.percentage);
    }

    public Command setStateCommand(IntakeState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(IntakeState.DISABLED);
    }

    private void updateSDB() {
        SmartDashboard.putBoolean("Intake/has note", isNoteInIntake());
    }

    @Override
    public void periodic() {
        if (UPDATE_DASHBOARD) {
            updateSDB();
        }
    }
}
