package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.motors.DBugSparkFlex;

import frc.robot.motors.PIDFGains;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private DBugSparkFlex _sparkFlexRightLeader;
    private DBugSparkFlex _sparkFlexLeftFollower;
    private ShooterState _shooterState;

    public static enum ShooterState {
        ON(ShooterConstants.SparkFlexShootingVelocity),
        OFF(ShooterConstants.SparkFlexUnShootingVelocity);

        public final double velocity;

        private ShooterState(double velocity) {
            this.velocity = velocity;
        }
    }

    public Shooter() {

        this._sparkFlexRightLeader = DBugSparkFlex.create(ShooterConstants.SparkFlexRightPort);
        this._sparkFlexLeftFollower = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);

        this._sparkFlexLeftFollower.setupPIDF(new PIDFGains(ShooterConstants.kpShooter, 0, 0, ShooterConstants.kfShooter));

        this._sparkFlexLeftFollower.follow(_sparkFlexRightLeader, true);

        // sets the shooter state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("State:", "OFF, enabled");
    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    public Command getSetStateCommand(ShooterState shooterState) {
        return new InstantCommand(() -> setState(shooterState), this);
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._sparkFlexRightLeader.setReference(shooterState.velocity, ControlType.kVelocity);

        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString());
    }

    public void stop() {
        this._shooterState = ShooterState.OFF;
        this._sparkFlexRightLeader.set(0);
    }

}
