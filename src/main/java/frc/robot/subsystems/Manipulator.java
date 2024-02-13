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
        OFF(ManipulatorConstants.offPercentage, ManipulatorConstants.offPercentage),
        COLLECT(ManipulatorConstants.leftCollectingPercentage, ManipulatorConstants.rightCollectingPercentage),
        AMP(ManipulatorConstants.AMPPercentage, ManipulatorConstants.AMPPercentage),
        TRAP(ManipulatorConstants.TRAPPercentage, ManipulatorConstants.TRAPPercentage),
        TO_SHOOTER(ManipulatorConstants.leftShooterPercentage, ManipulatorConstants.rightShooterPercentage);

        public double rightPercentage;
        public double leftPercentage;
        private ManipulatorState(double leftPercentage, double rightPercentage) {
            this.rightPercentage = rightPercentage;
            this.leftPercentage = leftPercentage;
        }
    }

    public Manipulator() {
        // TODO: check what ports it should be
        this._leader = DBugSparkMax.create(ManipulatorConstants.lowerSparkMaxPort);
        this._follower = DBugSparkMax.create(ManipulatorConstants.upperSparkMaxPort);
        this._leader.setSmartCurrentLimit(20);
        this._follower.setSmartCurrentLimit(20);
        
        this._hasNoteSwitch = new DigitalInput(ManipulatorConstants.noteSwitchPort);

        //this._follower.follow(this._leader, false);

        this._state = ManipulatorState.OFF;

        // initialize values into the SDB
        SmartDashboard.putString("Manipulator State", this._state.toString());
       
    }

    public ManipulatorState getManipulatorState() {
        return this._state;
    }

    // TODO: Check if NC or NO
    public boolean hasNoteSwitch() {
        return !_hasNoteSwitch.get();
    }

    private void setState(ManipulatorState state) {
        this._state = state;

        this._leader.set(state.leftPercentage);
        this._follower.set(state.rightPercentage);

        SmartDashboard.putString("Manipulator State", this._state.toString());
      
    }

    public Command getSetStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> setState(state), this);
    }

    public void stop() {
        setState(ManipulatorState.OFF);
    }

    public void manipulatByPercent(double follower,double leader){
        this._leader.set(leader);
        this._follower.set(follower);
    }

    @Override
    public void periodic() {
        // ManipulatorState.COLLECT.rightPercentage = SmartDashboard.getNumber("manipulator collect right", 0);
        // ManipulatorState.COLLECT.leftPercentage = SmartDashboard.getNumber("manipulator collect left", 0);
        // ManipulatorState.TO_SHOOTER.rightPercentage = SmartDashboard.getNumber("manipulator to shooter right", 0);
        // ManipulatorState.TO_SHOOTER.leftPercentage = SmartDashboard.getNumber("manipulator to shooter left", 0);
        //SmartDashboard.putNumber("manipulator collect", SmartDashboard.getNumber("manipulator collect", 0));
        //SmartDashboard.putNumber("manipulator to shooter", SmartDashboard.getNumber("manipulator to shooter", 0));
        SmartDashboard.putBoolean("has note", hasNoteSwitch());
        // SmartDashboard.putNumber("left current", _leader.getOutputCurrent());
        // SmartDashboard.putNumber("right current", _follower.getOutputCurrent());
        SmartDashboard.putNumber("right velocity rpm", _follower.getVelocity());
        SmartDashboard.putNumber("left velocity rpm", _leader.getVelocity());
        // SmartDashboard.putNumber("manipulator collect right", SmartDashboard.getNumber("manipulator collect right", 0));
        // SmartDashboard.putNumber("manipulator collect left", SmartDashboard.getNumber("manipulator collect left", 0));
        // SmartDashboard.putNumber("manipulator to shooter right", SmartDashboard.getNumber("manipulator to shooter right", 0));
        // SmartDashboard.putNumber("manipulator to shooter left", SmartDashboard.getNumber("manipulator to shooter left", 0));

     //   manipulatByPercent(SmartDashboard.getNumber("Manipulator Percentage leader", 0), SmartDashboard.getNumber("Manipulator Percentage follower", 0));
    }
}
