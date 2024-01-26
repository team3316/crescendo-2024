package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.motors.DBugSparkFlex;
import frc.robot.motors.PIDFGains;
import frc.robot.utils.Within;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    private DBugSparkFlex _leaderUpLeft;
    private DBugSparkFlex _followerUpRight;
    private DBugSparkFlex _followerDownLeft;
    private DBugSparkFlex _followerDownRight;
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

        this._leaderUpLeft = DBugSparkFlex.create(ShooterConstants.SparkFlexRightPort);
        this._followerUpRight = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);
        this._followerDownLeft = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);
        this._followerDownRight = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);

        this._leaderUpLeft.setupPIDF(new PIDFGains(ShooterConstants.kpShooter, 0, 0, ShooterConstants.kfShooter));

        this._followerUpRight.follow(_leaderUpLeft, true);
        this._followerDownLeft.follow(_leaderUpLeft, false);
        this._followerDownRight.follow(_leaderUpLeft, true);

        // sets the shooter state and the roller state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("shooter state:", "OFF");

    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._leaderUpLeft.setReference(shooterState.velocity, ControlType.kVelocity);

        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("shooter state:", this._shooterState.toString());
    }

    public Command getSetStateCommand(ShooterState targetState) {
        return new InstantCommand(() -> setState(targetState), this);
    }

    public double getMotorVelocity() {
        return this._leaderUpLeft.getVelocity();
    }

    public boolean isAtTargetVelocity() {
        return Within.range(this._shooterState.velocity, getMotorVelocity(), 0.01);
    }

    @Override
    public void periodic() {
        /* gets the velocity value from the SmartDashboard. To use these lines for calibration remove final keyword from 
        ShooterState.velocity and remember to put it back*/
        // ShooterState.ON.velocity = SmartDashboard.getNumber("velocity, rpm", 0);
        SmartDashboard.putNumber("Shooter velocity", getMotorVelocity());
        /*double curKP = SmartDashboard.getNumber("kp", 1);
        // only updates when the values are changed
        if (curKP != ShooterConstants.kpShooter){
            this._sparkFlexLeftFollower.setupPIDF(
                new PIDFGains(
                    curKP, 
                    0,
                    0,
                    ShooterConstants.kfShooter));
        }*/
    }
}
