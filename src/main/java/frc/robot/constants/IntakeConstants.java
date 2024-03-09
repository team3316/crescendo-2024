package frc.robot.constants;

import frc.robot.motors.PIDFGains;

public class IntakeConstants {
    public final static int intakeMotorID = 11;
    public static final int rollerID = 6;
    public final static int sensorID = 5;

    public final static double collectingPercentage = 1;
    public final static double ejectPercentage = -0.8;
    public final static double disabledPrecent = 0;

    public final static double intakeKp = 0;
    public final static double rollerkp = 0;

    public final static PIDFGains intakeGains = new PIDFGains(intakeKp, 0, 0, 0);
    public final static PIDFGains rollerGains = new PIDFGains(rollerkp, 0, 0, 0);

    public final static double intakeVelocityFactor = 0;//m/s
    public final static double rollerVelocityFactor = 0;//m/s

}
