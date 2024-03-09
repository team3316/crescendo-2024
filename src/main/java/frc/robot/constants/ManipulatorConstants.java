package frc.robot.constants;

import frc.robot.motors.PIDFGains;

public class ManipulatorConstants {
    // ports
    public static final int sparkmaxCANID = 10;
    public static final int noteSwitchPort = 8;

    // gains
    public static final PIDFGains positionGains = new PIDFGains(3.0); // TODO: Calibrate

    public static final double velocityKp = 2.5;
    public static final double velocityKi = 0;
    public static final double velocityKd = 0;
    public static final double velocityKf = 0.5/0.4;
    
    public final static PIDFGains velocityGains = new PIDFGains(velocityKp, 0, velocityKd, velocityKf);

    // positioning
    private static final double gearRatio = 1.0 / 9.0;
    private static final double rollerCircumference = Math.PI * 0.0254; // in meters
    public static final double positionFactor = rollerCircumference * gearRatio; // m/rotations
    public static final double velocityFactor = rollerCircumference * gearRatio / 60; // (m/sec)/RPM

    public class NotePosition { // TODO: Calibrate
        public static final double extract = 0.0; // extract from shooter. in meters
        public static final double amp = -0.25; // install in amp. in meters
        public static final double trap = -0.35; // install in trap. in meters
    }

    
    public static final double collectingVelocity = 0.4;
    
    // motor percentages
    public static final double offPercentage = 0;
    public static final double AMPPercentage = 1.0;
    public static final double TRAPPercentage = 0.5;
    public static final double toShooterPercentage = 0.35;
    public static final double collectingPercentage = 0.5;
    public static final double PreTrapPercentage = 0.2;
    public static final double ejectPercentage = -0.5;
}
