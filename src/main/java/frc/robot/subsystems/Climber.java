package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

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

    public void climb(double joystickPercentage) {
        Rotation2d gyroRotation2d = this._gyroSupplier.get();
        double error = gyroRotation2d.getSin() * ClimberConstants.spoolsDistance;

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
}
