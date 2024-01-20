package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.motors.DBugSparkFlex;

import frc.robot.motors.PIDFGains;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private DBugSparkFlex _sparkFlexRightLeader;
    private DBugSparkFlex _sparkFlexLeftFollower;
    private ShooterState _shooterState;
    private TalonSRX _shooterRoller;

    public enum ShooterState {
        ON(ShooterConstants.SparkFlexShootingVelocity, ShooterConstants.ShooterRollerPercent),
        OFF(ShooterConstants.SparkFlexUnShootingVelocity, 0 );

        public double velocity;
        public double ShooterRollerPercent;

        private ShooterState(double velocity, double ShooterRollerPercent) {
            this.velocity = velocity;
            this.ShooterRollerPercent = ShooterRollerPercent;
        }
    }

    public Shooter() {

        this._sparkFlexRightLeader = DBugSparkFlex.create(ShooterConstants.SparkFlexRightPort);
        this._sparkFlexLeftFollower = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);

        this._sparkFlexLeftFollower.setupPIDF(new PIDFGains(ShooterConstants.kpShooter, 0, 0, ShooterConstants.kfShooter));

        this._sparkFlexLeftFollower.follow(_sparkFlexRightLeader, true);

        this._shooterRoller = new TalonSRX(ShooterConstants.ShooterRollerPort);

        // sets the shooter state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("State:", "OFF, enabled");
    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    // TODO: check which is forward and which reverse, and whetehr they are NC or NO
    public boolean getSlowingLiftSwitch() {
        return _sparkFlexRightLeader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public boolean getStoppingLiftSwitch() {
        return _sparkFlexRightLeader.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._sparkFlexRightLeader.setReference(shooterState.velocity, ControlType.kVelocity);
        this._shooterRoller.set(TalonSRXControlMode.PercentOutput, this._shooterState.ShooterRollerPercent);
        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString());
    }

    public Command getSetStateCommand(ShooterState shooterState) {
        return new InstantCommand(() -> setState(shooterState), this);
    }



    @Override
    public void periodic() {;
        // gets the velocity value from the SmartDashboard
        this._shooterState.velocity = SmartDashboard.getNumber("velocity, rpm", 0);
        this._shooterState.ShooterRollerPercent = SmartDashboard.getNumber("Roller velocity, percentage", 0);

        // gets the kp value from the SmartDashboard
        this._sparkFlexLeftFollower.setupPIDF(
                new PIDFGains(SmartDashboard.getNumber("kp", 1),
                        0,
                        0,
                        ShooterConstants.kfShooter));

        //sets the velocity according to the new value
        setState(_shooterState);
    }

    // runs when is disable

    public void stop() {

        this._shooterState = ShooterState.OFF;
        setState(_shooterState);
        

        // prints the state change ont the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString() + ", disabled");

}}
