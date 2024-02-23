package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.motors.DBugSparkFlex;
import frc.robot.motors.PIDFGains;
import frc.robot.utils.Within;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.SnakeCaseStrategy;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    private DBugSparkFlex _leaderLeft;
    private DBugSparkFlex _followerRight;
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

        this._leaderLeft = DBugSparkFlex.create(ShooterConstants.leaderUpLeftPort,
                new PIDFGains(ShooterConstants.kpShooter, 0, 0, ShooterConstants.kfShooter),
                ShooterConstants.positionFactor, ShooterConstants.velocityFactor, 0);
        this._followerRight = DBugSparkFlex.create(ShooterConstants.followerDownRightPort);

        this._followerRight.follow(_leaderLeft, true);

        // sets the shooter state and the roller state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("shooter state:", "OFF");

    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._leaderLeft.setReference(shooterState.velocity, ControlType.kVelocity);

        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("shooter state:", this._shooterState.toString());
    }

    public Command getSetStateCommand(ShooterState targetState) {
        return new InstantCommand(() -> setState(targetState), this);
    }

    public Command getShooterSpinCommand(){
        return new StartEndCommand(() -> setState(ShooterState.ON), () -> setState(ShooterState.OFF), this);
    }

    private double getShooterVelocityMPS() {
        return this._leaderLeft.getVelocity();
    }

    public boolean isAtTargetVelocity() {
        return Within.range(this._shooterState.velocity, getShooterVelocityMPS(), 0.1);
    }

    @Override
    public void periodic() {
        /*
         * gets the velocity value from the SmartDashboard. To use these lines for
         * calibration remove final keyword from
         * ShooterState.velocity and remember to put it back
         */
        /*ShooterState.ON.velocity = SmartDashboard.getNumber("velocity, mps", 0);
        if (DriverStation.isEnabled()) {
            // setState(ShooterState.ON);
            // leaderUpLeft.set(0.5);
        }
        SmartDashboard.putNumber("velocity, mps", SmartDashboard.getNumber("velocity, mps", 0));*/
        SmartDashboard.putNumber("Shooter velocity", getShooterVelocityMPS());
        SmartDashboard.putNumber("shooter BL current", _leaderLeft.getOutputCurrent());
        SmartDashboard.putNumber("shooter BR current", _followerRight.getOutputCurrent());
        SmartDashboard.putBoolean("Is at target velocity", isAtTargetVelocity());

    }

    public void stop() {
        _leaderLeft.set(0);
    }
}
