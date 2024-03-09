package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

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

        private IntakeState(double percentage) {
            this.percentage = percentage;
        }
    }

    public void upDatePIDF(double intakeKp, double intakeKi, double intakeKd) {
        this._intakeMotor.setupPIDF(new PIDFGains(intakeKp, intakeKi, intakeKd, IntakeConstants.intakeKf));
    }

    public Intake() {
        this._hasNoteSwitch = new DigitalInput(IntakeConstants.sensorID);

        _intakeMotor = DBugSparkMax.create(IntakeConstants.intakeMotorID, IntakeConstants.intakeGains,
                IntakeConstants.intakePositionFactor,
                IntakeConstants.intakeVelocityFactor, 0);
        _intakeMotor.setSmartCurrentLimit(15);

        // _intakeRoller =
        // DBugSparkMax.create(IntakeConstants.rollerID,IntakeConstants.rollerGains,
        // IntakeConstants.rollerPositionFactor,
        // IntakeConstants.rollerVelocityFactor, 0);
        _intakeRoller = DBugSparkMax.create(IntakeConstants.rollerID);
        _intakeRoller.setSmartCurrentLimit(15);

        this._state = IntakeState.DISABLED;

        SmartDashboard.putNumber("Intake/error", -1);
         SmartDashboard.putData("Intake/intakePID",new InstantCommand(()-> new InstantCommand(() -> upDatePIDF(SmartDashboard.getNumber("Intake/intakeKp", 0),
                SmartDashboard.getNumber("Intake/intakeKi", 0), SmartDashboard.getNumber("Intake/intakeKd", 0))).schedule()));


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

    public double getVelocity() {
        return _intakeMotor.getVelocity();
    }

    public Command setStateCommand(IntakeState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void setToCollect(){
        this._intakeMotor.setReference(IntakeConstants.collectingVelocity, ControlType.kVelocity);
        this._intakeRoller.set(IntakeConstants.collectingPercentage);
    }

    public Command getCollectCommand(){
        return new InstantCommand(() -> setToCollect(), this);
    }

    public void stop() {
        _intakeMotor.set(0);
        _intakeRoller.set(0);
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Intake/intakeKp", SmartDashboard.getNumber("Intake/intakeKp", IntakeConstants.intakeKp));
        SmartDashboard.putNumber("Intake/intakeKi", SmartDashboard.getNumber("Intake/intakeKi", IntakeConstants.intakeKi));
        SmartDashboard.putNumber("Intake/intakeKd", SmartDashboard.getNumber("Intake/intakeKd", IntakeConstants.intakeKd));

       
        SmartDashboard.putNumber("Intake/error", getVelocity() - IntakeConstants.collectingVelocity);
        SmartDashboard.putBoolean("Intake/has note", isNoteInIntake());
    }

    @Override
    public void periodic() {
        if (UPDATE_DASHBOARD) {
            updateSDB();
        }
    }
}
