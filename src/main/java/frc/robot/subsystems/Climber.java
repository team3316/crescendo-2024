package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import frc.robot.utils.Within;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class Climber extends SubsystemBase {
    private DBugSparkMax _leftSpool;
    private DBugSparkMax _rightSpool;
    private Supplier<Rotation2d> _gyroSupplier;
    private PIDController _balanceController;

    public Climber(Supplier<Rotation2d> gyroSupplier) {
        this._leftSpool = DBugSparkMax.create(ClimberConstants.leftSpoolPort);
        this._rightSpool = DBugSparkMax.create(ClimberConstants.rightSpoolPort);
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

    private void climb() {
        Rotation2d gyroRotation2d = this._gyroSupplier.get();
        double balancePID = _balanceController.calculate(gyroRotation2d.getSin() * ClimberConstants.spoolsDistance);

        _leftSpool.set(ClimberConstants.feedforward - balancePID);
        _rightSpool.set(ClimberConstants.feedforward + balancePID);
    }

    public void setPercentage(double left, double right) {
        _leftSpool.set(left);
        _rightSpool.set(right);
    }

    public Command getClimbCommand() {
        return new RunCommand(this::climb, this);
    }

    public void stop() {
        _leftSpool.set(0);
        _rightSpool.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber left current", _leftSpool.getOutputCurrent());
        SmartDashboard.putNumber("climber right current", _rightSpool.getOutputCurrent());    
    }
}
