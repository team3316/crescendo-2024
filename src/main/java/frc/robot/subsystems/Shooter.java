package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.constantsShooter;
import frc.robot.motors.DBugSparkFlex;

import frc.robot.motors.PIDFGains;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase{
    private DBugSparkFlex _SparkFlexRightLeader;
    private DBugSparkFlex _SparkFlexLeftFollower;
    private ShooterState _ShooterState;

    private SimpleMotorFeedforward feedForward;
    

    public static enum ShooterState{
        ON(constantsShooter.SparFlexShootingVelocity),
        OFF(constantsShooter.SparkFlexUnShootingVelocity);
        public final double v;

        private ShooterState(double v){
            this.v = v;
        }
    }

    public Shooter(){
        
        _SparkFlexRightLeader = DBugSparkFlex.create(constantsShooter.SparkFlexRightPort);
        _SparkFlexLeftFollower = DBugSparkFlex.create(constantsShooter.SparkFlexLeftPort);

        _SparkFlexLeftFollower.setupPIDF(new PIDFGains(constantsShooter.kpShooter));
        

        _SparkFlexLeftFollower.follow(_SparkFlexRightLeader, true);

        feedForward = new SimpleMotorFeedforward(0, constantsShooter.kvFeedForwardShooter);


    }

    public ShooterState getShooterState(){
        return this._ShooterState;
    }

    public Command getSetStateCommand(ShooterState shooterState){
        return new InstantCommand(() -> setState(shooterState), this);

        
    }

    private void setState(ShooterState shooterState){
        _ShooterState = shooterState;
        _SparkFlexRightLeader.setReference(shooterState.v, ControlType.kVelocity, 0, feedForward.calculate(shooterState.v), ArbFFUnits.kVoltage);
        SmartDashboard.putString("State:",_ShooterState.toString());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("velocity, rpm", _SparkFlexRightLeader.getVelocity());

    }

}
