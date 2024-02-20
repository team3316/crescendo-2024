package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import frc.robot.utils.Within;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class Climber extends SubsystemBase {
    private DBugSparkMax _leftSpool, _rightSpool;
    private Supplier<Rotation2d> _gyroSupplier;
    private PIDController _balanceController;

    public Climber(Supplier<Rotation2d> gyroSupplier) {
        this._leftSpool = DBugSparkMax.create(ClimberConstants.leftSpoolPort, 
                new PIDFGains(ClimberConstants.averageKp, 0, 0, ClimberConstants.averageKf),
                ClimberConstants.positionFactor, ClimberConstants.velocityFactor, 0);
        this._rightSpool = DBugSparkMax.create(ClimberConstants.rightSpoolPort, 
                new PIDFGains(ClimberConstants.averageKp, 0, 0, ClimberConstants.averageKf),
                ClimberConstants.positionFactor, ClimberConstants.velocityFactor, 0);
        this._rightSpool.setInverted(true);
        

        this._gyroSupplier = gyroSupplier;

        this._balanceController = new PIDController(ClimberConstants.differentialKp, 0, 0);
        this._balanceController.setSetpoint(0);
    }

    public double getLeftPosition() {
        return this._leftSpool.getPosition();
    }

    public double getRightPosition() {
        return this._rightSpool.getPosition();
    }


    private double weakenIfPositive(double value) {
        if (value > 0) {
            return value * ClimberConstants.differentialRatio;
        }
        return value;
    }

    private void climb() {
        Rotation2d gyroRotation2d = this._gyroSupplier.get();
        double balanceFeedforward = _balanceController.calculate(gyroRotation2d.getSin() * ClimberConstants.spoolsDistance);

        // TODO: check gyro direction
        this._leftSpool.setReference(ClimberConstants.climbHeight, ControlType.kPosition, 0,
                weakenIfPositive(balanceFeedforward), ArbFFUnits.kPercentOut);
        this._rightSpool.setReference(ClimberConstants.climbHeight, ControlType.kPosition, 0,
                weakenIfPositive(-balanceFeedforward), ArbFFUnits.kPercentOut);

        
    }

    public void setPercentage(double left, double right) {
        _leftSpool.set(left);
        _rightSpool.set(right);
    }

    public Command getClimbCommand() {
        BooleanSupplier atHeight = () ->Within.range(getLeftPosition(), ClimberConstants.climbHeight, ClimberConstants.climbTolerance);
        return new FunctionalCommand(() -> {}, this::climb, (interrupted) -> stop(), atHeight);
    }

    public void stop() {
        _leftSpool.set(0);
        _rightSpool.set(0);
    }

    @Override
    public void periodic() {
        
    }
}
