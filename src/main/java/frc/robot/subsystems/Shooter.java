package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.constantsShooter;
import frc.robot.motors.DBugSparkFlex;

import frc.robot.motors.PIDFGains;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private DBugSparkFlex _sparkFlexRightLeader;
    private DBugSparkFlex _sparkFlexLeftFollower;
    private ShooterState _shooterState;

    private static double kpShooter = 1;

    public static enum ShooterState {
        ON(constantsShooter.SparkFlexShootingVelocity),
        OFF(constantsShooter.SparkFlexUnShootingVelocity);

        public double velocity;

        private ShooterState(double velocity) {
            this.velocity = velocity;
        }
    }

    public Shooter() {

        this._sparkFlexRightLeader = DBugSparkFlex.create(constantsShooter.SparkFlexRightPort);
        this._sparkFlexLeftFollower = DBugSparkFlex.create(constantsShooter.SparkFlexLeftPort);

        this._sparkFlexLeftFollower.setupPIDF(new PIDFGains(kpShooter, 0, 0, constantsShooter.kfShooter));

        this._sparkFlexLeftFollower.follow(_sparkFlexRightLeader, true);

        // sets the shooter state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("State:", "OFF, enabled");
    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    public Command getSetStateCommand(ShooterState shooterState) {
        // only sets the Shooter's state, doesn't stops shooting independently
        return new InstantCommand(() -> setState(shooterState), this);
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._sparkFlexRightLeader.setReference(shooterState.velocity, ControlType.kVelocity);

        // prints the state change ont the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString());
    }

    @Override
    public void periodic() {;
        // gets the velocity value from the SmartDashboard
        this._shooterState.velocity = SmartDashboard.getNumber("velocity, rpm", 0);

        // gets the kp value from the SmartDashboard
        this._sparkFlexLeftFollower.setupPIDF(
                new PIDFGains(SmartDashboard.getNumber("kp", 1),
                        0,
                        0,
                        constantsShooter.kfShooter));
    }

    // runs when is disable
    public void disableInit() {
        this._shooterState = ShooterState.OFF;
        this._sparkFlexRightLeader.setReference(this._shooterState.velocity, ControlType.kVelocity);

        // prints the state change ont the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString() + ", disabled");
    }

}
