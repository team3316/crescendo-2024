package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.constantsShooter;
import frc.robot.motors.DBugSparkMax;

public class Shooter extends SubsystemBase{
    private DBugSparkMax _SparkMaxRightLeader;
    private DBugSparkMax _SparkMaxLeftFollower;

    public static enum ShooterState{
        ON(constantsShooter.SparkMaxShootingPercent),
        OFF(0);
        public final double percentage;

        private ShooterState(double percentage){
            this.percentage = percentage;
        }
    }

    public Shooter(){
        
        _SparkMaxRightLeader = new DBugSparkMax(constantsShooter.SparkMaxRightPort);
        _SparkMaxLeftFollower = new DBugSparkMax(constantsShooter.SpatkMaxLeftPort);

        _SparkMaxLeftFollower.follow(_SparkMaxRightLeader);



    }
}
