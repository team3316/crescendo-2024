package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Arm extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;

    private ArmFeedforward _feedforward;

    public static enum ArmState {
        COLLECT(ArmConstants.collectAngle),
        AMP(ArmConstants.AMPAngle),
        TRAP(ArmConstants.TRAPAngle),
        CLIMB(ArmConstants.climbAngle);

        public final double angleDeg;

        private ArmState(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }

    public Arm() {
        _leader = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _follower = DBugSparkMax.create(ArmConstants.leaderCANID, new PIDFGains(ArmConstants.kp), 
        ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);

        _feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);
    }


}
