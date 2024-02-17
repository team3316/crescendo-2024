package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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
        _leftSpool.setInverted(true);
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

    private void climb() {
        Rotation2d gyroRotation2d = this._gyroSupplier.get();
        double balanceFeedforward = _balanceController.calculate(gyroRotation2d.getSin() * ClimberConstants.spoolsDistance);

        // TODO: check gyro direction
        this._leftSpool.setReference(ClimberConstants.climbHeight, ControlType.kPosition, 0,
                balanceFeedforward, ArbFFUnits.kPercentOut);
        this._rightSpool.setReference(ClimberConstants.climbHeight, ControlType.kPosition, 0,
                -balanceFeedforward, ArbFFUnits.kPercentOut);
        this._leftSpool.setInverted(true);

        // putting values into the Smart Dashboard
        SmartDashboard.putNumber("Error Angle, degrees", gyroRotation2d.getDegrees());
        SmartDashboard.putNumber("Left Spool Position, meters", getLeftPosition());
        SmartDashboard.putNumber("Right Spool Position, meters", getRightPosition());
    }

    public Command getClimbCommand() {
        return new RunCommand(this::climb, this);
    }

    public void setPercentage(double left, double right) {
        _leftSpool.set(left);
        _rightSpool.set(right);
    }

    public void upClimbPercentage() {
        setPercentage(SmartDashboard.getNumber("leftUp", 0), SmartDashboard.getNumber("rightUp", 0));
    }

    public void downClimbPercentage() {
        setPercentage(SmartDashboard.getNumber("leftDown", 0), SmartDashboard.getNumber("rightDown", 0));
    }

    public void stop() {
        _leftSpool.set(0);
        _rightSpool.set(0);
    }

    public void brake(){
        this._leftSpool.setIdleMode(IdleMode.kBrake);
        this._rightSpool.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftUp", SmartDashboard.getNumber("leftUp", 0));
        SmartDashboard.putNumber("rightUp", SmartDashboard.getNumber("rightUp", 0));
        SmartDashboard.putNumber("leftDown", SmartDashboard.getNumber("leftDown", 0));
        SmartDashboard.putNumber("rightDown", SmartDashboard.getNumber("rightDown", 0));
        //upClimbPercentage();
    }
}
