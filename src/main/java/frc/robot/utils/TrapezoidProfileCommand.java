package frc.robot.utils;

import java.util.function.Consumer;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapezoidProfileCommand extends Command {

    private TrapezoidProfile _profile;
    private Consumer<State> _stateUser;
    private Timer _timer;
    private State _startState, _goalState;

    public TrapezoidProfileCommand(TrapezoidProfile profile, Consumer<State> stateUser, State current, State goal, SubsystemBase... requirements) {
        this._profile = profile;
        this._stateUser = stateUser;
        addRequirements(requirements);
        _timer = new Timer();
    }

    @Override
    public void initialize() {
        _timer.start();
    }

    @Override
    public void execute() {
        State currentGoalState = _profile.calculate(_timer.get(), _startState, _goalState);
        _stateUser.accept(currentGoalState);
    }

    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(_profile.totalTime());
    }

    @Override
    public void end(boolean interrupted) {
        _timer.stop();
        _timer.reset();
    }
}
