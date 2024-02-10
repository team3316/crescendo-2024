package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.motors.DBugSparkMax;

public class Manipulator extends SubsystemBase {

    private DBugSparkMax _leader;
    private DBugSparkMax _follower;

    private DigitalInput _hasNoteSwitch;

    private ManipulatorState _state;

    public static enum ManipulatorState {
        OFF(ManipulatorConstants.offPercentage),
        COLLECT(ManipulatorConstants.collectingPercentage),
        AMP(ManipulatorConstants.AMPPercentage),
        TRAP(ManipulatorConstants.TRAPPercentage),
        TO_SHOOTER(ManipulatorConstants.shooterPercentage);

        public double percentage;
        private ManipulatorState(double percentage) {
            this.percentage = percentage;
        }
    }

    public Manipulator() {
        // TODO: check what ports it should be
        this._leader = DBugSparkMax.create(ManipulatorConstants.lowerSparkMaxPort);
        this._follower = DBugSparkMax.create(ManipulatorConstants.upperSparkMaxPort);
        this._leader.setSmartCurrentLimit(20);
        this._follower.setSmartCurrentLimit(20);
        
        this._hasNoteSwitch = new DigitalInput(ManipulatorConstants.noteSwitchPort);

        this._follower.follow(this._leader, false);

        this._state = ManipulatorState.OFF;

        // initialize values into the SDB
        SmartDashboard.putString("Manipulator State", this._state.toString());
        SmartDashboard.putNumber("Manipulator Percentage", this._state.percentage);
    }

    public ManipulatorState getManipulatorState() {
        return this._state;
    }

    // TODO: Check if NC or NO
    public boolean hasNoteSwitch() {
        return _hasNoteSwitch.get();
    }

    private void setState(ManipulatorState state) {
        this._state = state;

        this._leader.set(this._state.percentage);

        SmartDashboard.putString("Manipulator State", this._state.toString());
        SmartDashboard.putNumber("Manipulator Percentage", this._state.percentage);
    }

    public Command getSetStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(ManipulatorState.OFF);
    }

    @Override
    public void periodic() {
        ManipulatorState.COLLECT.percentage = SmartDashboard.getNumber("manipulator collect", 0);
        ManipulatorState.TO_SHOOTER.percentage = SmartDashboard.getNumber("manipulator to shooter", 0);
        SmartDashboard.putNumber("mainpulator collect", SmartDashboard.getNumber("manipulator collect", 0));
        SmartDashboard.putNumber("manipulator to shooter", SmartDashboard.getNumber("manipulator to shooter", 0));
    }
}
