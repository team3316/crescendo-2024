package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.motors.DBugSparkMax;

public class Intake extends SubsystemBase {
    
    private DBugSparkMax _intakeMotor;

    private IntakeState _state;

    //The diffrent states of the intake.
    public static enum IntakeState {
        COLLECTING(IntakeConstants.collectingPrecent),
        DISABLED(IntakeConstants.disabledPrecent);
        public final double precentege;

        private IntakeState(double precentege){
            this.precentege = precentege;
        }
    }

    public Intake(){
        _intakeMotor = DBugSparkMax.create(IntakeConstants.intakeMotorID);

        this._state = IntakeState.DISABLED; 
    }

    public IntakeState getState(){
        return this._state;
    }

    public void setState(IntakeState state){
        this._state = state;

        _intakeMotor.set(state.precentege);
        SmartDashboard.putString("Intake State: ", this._state.toString());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Motor state precentage ", this._state.precentege);
    }
    
    public Command setStateCommand(IntakeState state){
        return new InstantCommand(() -> setState(state),this);
    }
}
