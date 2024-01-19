package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import com.revrobotics.CANSparkBase.ControlType;

public class Climber extends SubsystemBase {
    public static enum ClimberState {
        START(ClimberConstants.startingVelocity),
        INTERMEDIATE(0), // value don't change anything.
        END(ClimberConstants.endingVelocity);

        public final double velocity;

        private ClimberState(double velocity) {
            this.velocity = velocity;
        }
    }

    private DBugSparkMax _sparkMax;
    private ClimberState _state;

    public Climber() {
        this._sparkMax = DBugSparkMax.create(ClimberConstants.sparkMaxPort);
        this._state = ClimberState.START;
        this._sparkMax.setPosition(this._state.velocity);
        this._sparkMax.setupPIDF(new PIDFGains(ClimberConstants.kP, 0, 0, ClimberConstants.kF));
    }

    // if you call the {setState(state)} method, until the motor action ends,
    // the {getState()} method will return {ClimberState.INTERMEDIATE},
    // but the {getPreferredState()} method will return {state}.
    // when the motor action ends, both will return {state}.
    public ClimberState getState() {
        if (this._sparkMax.get() == ClimberState.START.velocity)
            return ClimberState.START;
        if (this._sparkMax.get() == ClimberState.END.velocity)
            return ClimberState.END;
        return ClimberState.INTERMEDIATE;
    }

    public ClimberState getPreferredState() {
        return this._state;
    }

    private void setState(ClimberState state) {
        this._state = state;
        this._sparkMax.setReference(this._state.velocity, ControlType.kVelocity);
    }

    public Command getSetStateCommand(ClimberState state) {
        return new InstantCommand(() -> setState(state));
    }

    @Override
    public void periodic() {
        // update the smart dashboard
        SmartDashboard.putNumber("Climber Progress, percent", this._sparkMax.get());
        SmartDashboard.putString("Climber State",
                "Current: " + getState().toString() + ", Preferred: " + getPreferredState());

        // get the preferred kP values from the smart dashboard
        this._sparkMax.setupPIDF(
                new PIDFGains(SmartDashboard.getNumber("kP", 1),
                        0,
                        0,
                        ClimberConstants.kF));

        this._sparkMax.setReference(SmartDashboard.getNumber("Velocity, rmp", this._state.velocity), ControlType.kVelocity);
    }
}
