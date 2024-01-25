package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ShooterConstants;
import frc.robot.motors.DBugSparkFlex;
import frc.robot.motors.PIDFGains;
import frc.robot.utils.Within;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    private DBugSparkFlex _leader;
    private DBugSparkFlex _follower;
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

        this._leader = DBugSparkFlex.create(ShooterConstants.SparkFlexRightPort);
        this._follower = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);

        this._leader.setupPIDF(new PIDFGains(ShooterConstants.kpShooter, 0, 0, ShooterConstants.kfShooter));

        this._follower.follow(_leader, true);

        // sets the shooter state and the roller state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("shooter state:", "OFF");

    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    // TODO: check which is forward and which reverse, and whetehr they are NC or NO
    public boolean getSlowingLiftSwitch() {
        return _leader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public boolean getStoppingLiftSwitch() {
        return _leader.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._leader.setReference(shooterState.velocity, ControlType.kVelocity);

        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("shooter state:", this._shooterState.toString());
    }

    public Command getSetStateCommand(ShooterState targetState) {
        return new InstantCommand(() -> setState(targetState), this);
    }

    public double getMotorVelocity() {
        return this._leader.getVelocity();
    }

    public boolean isAtTargetVelocity() {
        return Within.range(this._shooterState.velocity, getMotorVelocity(), 0.01);
    }

    @Override
    public void periodic() {
        /* gets the velocity value from the SmartDashboard. To use these lines for calibration remove final keyword from 
        ShooterState.velocity & RollerState.ShooterRollerPercent and remember to put it back*/
        // ShooterState.ON.velocity = SmartDashboard.getNumber("velocity, rpm", 0);
        // RollerState.ON.ShooterRollerPercent = SmartDashboard.getNumber("Roller velocity, percentage", 0);
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
