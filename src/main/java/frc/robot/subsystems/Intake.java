package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.motors.DBugSparkMax;

public class Intake extends SubsystemBase {
    private DBugSparkMax _intakeMotor;

    private CollectorState _state;

    public enum CollectorState {
        COLLECTING(IntakeConstants.collectingPrecent),
        DISABLED(IntakeConstants.disabledPrecent);
        public final double precentege;

        private CollectorState(double precentege){
            this.precentege = precentege;
        }
    }

    public Intake(){
        _intakeMotor = new DBugSparkMax(IntakeConstants.intakeMotorID);

        this._state = CollectorState.DISABLED; 
    }

    public CollectorState getState(){
        return this._state;
    }

    public void setState(CollectorState state){
        this._state = state;

        _intakeMotor.setReference(state.precentege, ControlType.kVelocity);
    }
}
