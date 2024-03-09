package frc.robot.constants;

import frc.robot.motors.PIDFGains;

public class ManipulatorConstants {
    // ports
    public static final int sparkmaxCANID = 10;
    public static final int noteSwitchPort = 8;

    // gains
    public static final PIDFGains positionGains = new PIDFGains(3.0); // TODO: Calibrate

    private static final double velocityKp = 0;
    public final static PIDFGains velocityGains = new PIDFGains(velocityKp);

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

    // motor velocity
    public static final double offVelocity = 0;
    public static final double AMPVelocity = 0;
    public static final double TRAPVelocity = 0;
    public static final double toShooterVelocity = 0;
    public static final double collectingVelocity = 0;
    public static final double PreTrapVelocity = 0;
    public static final double ejectVelocity = 0;
}
