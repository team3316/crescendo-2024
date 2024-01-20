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

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private DBugSparkFlex _sparkFlexRightLeader;
    private DBugSparkFlex _sparkFlexLeftFollower;
    private ShooterState _shooterState;
    private RollerState _rollerState;
    private TalonSRX _shooterRoller;

    public enum ShooterState {
        ON(ShooterConstants.SparkFlexShootingVelocity),
        OFF(ShooterConstants.SparkFlexUnShootingVelocity);

        public double velocity;

        private ShooterState(double velocity) {
            this.velocity = velocity;
        }
    }

    public enum RollerState {
        ON( ShooterConstants.ShooterRollerPercent),
        OFF(0 );

        public double ShooterRollerPercent;

        private RollerState(double ShooterRollerPercent) {
            this.ShooterRollerPercent = ShooterRollerPercent;
        }
    }

    public Shooter() {

        this._sparkFlexRightLeader = DBugSparkFlex.create(ShooterConstants.SparkFlexRightPort);
        this._sparkFlexLeftFollower = DBugSparkFlex.create(ShooterConstants.SparkFlexLeftPort);

        this._sparkFlexLeftFollower.setupPIDF(new PIDFGains(ShooterConstants.kpShooter, 0, 0, ShooterConstants.kfShooter));

        this._sparkFlexLeftFollower.follow(_sparkFlexRightLeader, true);

        this._shooterRoller = new TalonSRX(ShooterConstants.ShooterRollerPort);

        // sets the shooter state and the roller state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("shooter state:", "OFF, enabled");

        this._rollerState = RollerState.OFF;
        SmartDashboard.putString("roller state:", "OFF, enabled");
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

        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("shooter state:", this._shooterState.toString());
    }

    private void setRollerState(RollerState rollerState){
        this._shooterRoller.set(TalonSRXControlMode.PercentOutput, this._rollerState.ShooterRollerPercent);
        // prints the state change onto the SmartDashboard
        SmartDashboard.putString("roller state:", this._rollerState.toString());
    }

    public Command getSetStateCommand(ShooterState shooterState) {
        return new InstantCommand(() -> setState(shooterState), this);
    }

    public Command getShootingCommand() {
        Command shootingSeq = Commands.sequence(
            new InstantCommand(() -> setState(ShooterState.ON), this),
            new WaitUntilCommand(this::isAtTargetVelocity),
            new InstantCommand(() -> setRollerState(RollerState.ON), this),
            new WaitCommand(ShooterConstants.shootingTime),
            new InstantCommand(() -> {
                setState(ShooterState.OFF);
                setRollerState(RollerState.OFF);
            }, this)
        );
        Command toReturn = new ConditionalCommand(shootingSeq, new InstantCommand(), this::getStoppingLiftSwitch);
        toReturn.addRequirements(this);
        return toReturn;
    }

    public double getMotorVelocity() {
        return this._sparkFlexRightLeader.getVelocity();
    }

    public boolean isAtTargetVelocity() {
        return getMotorVelocity() == this._shooterState.velocity;
    }



    @Override
    public void periodic() {;
        // gets the velocity value from the SmartDashboard
        this._shooterState.velocity = SmartDashboard.getNumber("velocity, rpm", 0);
        this._rollerState.ShooterRollerPercent = SmartDashboard.getNumber("Roller velocity, percentage", 0);

        // gets the kp value from the SmartDashboard
        this._sparkFlexLeftFollower.setupPIDF(
                new PIDFGains(SmartDashboard.getNumber("kp", 1),
                        0,
                        0,
                        ShooterConstants.kfShooter));
    }

    // runs when it is disabled
    public void stop() {

        this._shooterState = ShooterState.OFF;
        setState(_shooterState);


        // prints the state change ont the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString() + ", disabled");

}}
