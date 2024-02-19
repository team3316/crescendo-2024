package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.motors.DBugSparkMax;

public class Intake extends SubsystemBase {
    
    private DBugSparkMax _intakeMotor;

    private DigitalInput _hasNoteSwitch;

    private IntakeState _state;

    //The diffrent states of the intake.
    public static enum IntakeState {
        COLLECTING(IntakeConstants.collectingPercentage),
        EJECT(IntakeConstants.ejectPercentage),
        DISABLED(IntakeConstants.disabledPrecent);
        public final double percentage;

        private IntakeState(double precentege){
            this.percentage = precentege;
        }
    }

    public Intake(){
                this._hasNoteSwitch = new DigitalInput(IntakeConstants.sensor_port);

        _intakeMotor = DBugSparkMax.create(IntakeConstants.intakeMotorID);
        _intakeMotor.setSmartCurrentLimit(15);

        this._state = IntakeState.DISABLED; 
    }

    public boolean isNoteInIntake(){
        return !_hasNoteSwitch.get();

    }

    public IntakeState getState(){
        return this._state;
    }

    private void setState(IntakeState state){
        this._state = state;

        _intakeMotor.set(state.percentage);
        SmartDashboard.putString("Intake State: ", this._state.toString());
        //SmartDashboard.putNumber("Intake precentage ", this._state.percentage);
    }
    
    public Command setStateCommand(IntakeState state){
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(IntakeState.DISABLED);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("intake percentage", SmartDashboard.getNumber("intake percentage", 0));
        // IntakeState.COLLECTING.percentage = SmartDashboard.getNumber("intake percentage", 0);
        SmartDashboard.putNumber("Intake velocity rpm", _intakeMotor.getVelocity());
        SmartDashboard.putBoolean("intake switch", isNoteInIntake());
    }
}
