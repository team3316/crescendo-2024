package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.constantsShooter;
import frc.robot.motors.DBugSparkFlex;

import frc.robot.motors.PIDFGains;


import com.revrobotics.CANSparkBase.ControlType;




import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private DBugSparkFlex _sparkFlexRightLeader;
    private DBugSparkFlex _sparkFlexLeftFollower;
    private ShooterState _shooterState;

    private static double SparFlexShootingVelocity; // rpm

    private static double kpShooter = 1;

    

    public static enum ShooterState {
        ON(SparFlexShootingVelocity),
        OFF(constantsShooter.SparkFlexUnShootingVelocity);
        public final double velocity;

        private ShooterState(double velocity) {
            this.velocity = velocity;
        }
    }

    public Shooter() {
        
        this._sparkFlexRightLeader = DBugSparkFlex.create(constantsShooter.SparkFlexRightPort);
        this._sparkFlexLeftFollower = DBugSparkFlex.create(constantsShooter.SparkFlexLeftPort);

        this._sparkFlexLeftFollower.setupPIDF(new PIDFGains(kpShooter, 0, 0, constantsShooter.kfShooter));
        

        this._sparkFlexLeftFollower.follow(_sparkFlexRightLeader, true);

        // sets the shooter state to OFF in the beginning
        this._shooterState = ShooterState.OFF;
        SmartDashboard.putString("State:", "OFF, enabled");
    }

    public ShooterState getShooterState() {
        return this._shooterState;
    }

    public Command getSetStateCommand(ShooterState shooterState) {
        // only sets the Shooter's state, doesn't stops shooting independently
        return new InstantCommand(() -> setState(shooterState), this);
    }

    private void setState(ShooterState shooterState) {
        this._shooterState = shooterState;
        this._sparkFlexRightLeader.setReference(shooterState.velocity, ControlType.kVelocity);

        SmartDashboard.putString("State:", this._shooterState.toString());
        // prints the state change ont the SmartDashboard
    }

    @Override
    public void periodic() {
        // prints the velocity to the SmartDashboard
        SmartDashboard.putNumber("velocity, rpm", this._sparkFlexRightLeader.getVelocity());

        // gets the velocity value from the SmartDashboard
        SparFlexShootingVelocity = SmartDashboard.getNumber("velocity", 0);

        // gets the kp value from the SmartDashboard
        kpShooter = SmartDashboard.getNumber("kp", 1);
    }

    // runs when is disable
    public void disableInit() {
        this._shooterState = ShooterState.OFF;
        this._sparkFlexRightLeader.setReference(this._shooterState.velocity, ControlType.kVelocity);

        // prints the state change ont the SmartDashboard
        SmartDashboard.putString("State:", this._shooterState.toString() + ", disabled");
    }

}
