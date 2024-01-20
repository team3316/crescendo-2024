package frc.robot.subsystems;

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
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;

public class Climber extends SubsystemBase {
    private DBugSparkMax _leftSpool, _rightSpool;
    private Supplier<Rotation2d> _gyroSupplier;

    public Climber(Supplier<Rotation2d> gyroSupplier) {
        this._leftSpool = DBugSparkMax.create(ClimberConstants.leftSpoolPort, new PIDFGains(ClimberConstants.kP),
                ClimberConstants.positionFactor, ClimberConstants.velocityFactor, 0);
        this._rightSpool = DBugSparkMax.create(ClimberConstants.rightSpoolPort, new PIDFGains(ClimberConstants.kP),
                ClimberConstants.positionFactor, ClimberConstants.velocityFactor, 0);

        this._gyroSupplier = gyroSupplier;
    }

    public double getLeftPosition() {
        return this._leftSpool.getPosition();
    }

    public double getRightPosition() {
        return this._rightSpool.getPosition();
    }

    private void climb(Supplier<Double> joystickPercentageSupplier) {
        Rotation2d gyroRotation2d = this._gyroSupplier.get();
        double error = gyroRotation2d.getSin() * ClimberConstants.spoolsDistance;
        double joystickPercentage = joystickPercentageSupplier.get();

        // TODO: check gyro direction
        this._leftSpool.setReference(getLeftPosition() + (error / 2), ControlType.kPosition, 0,
                joystickPercentage, ArbFFUnits.kPercentOut);
        this._rightSpool.setReference(getRightPosition() - (error / 2), ControlType.kPosition, 0,
                joystickPercentage, ArbFFUnits.kPercentOut);

        // putting values into the Smart Dashboard
        SmartDashboard.putNumber("Error Angle, degrees", gyroRotation2d.getDegrees());
        SmartDashboard.putNumber("Joystick Percentage", joystickPercentage);
        SmartDashboard.putNumber("Left Spool Position, meters", getLeftPosition());
        SmartDashboard.putNumber("Right Spool Position, meters", getRightPosition());
    }

    public Command getClimbCommand(Supplier<Double> joystickPercentageSupplier) {
        return new RunCommand(() -> climb(joystickPercentageSupplier), this);
    }
}
