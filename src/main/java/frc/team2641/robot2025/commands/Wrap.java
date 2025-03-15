package frc.team2641.robot2025.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.climber.Winch;

public class Wrap extends Command {
  private Winch winch = Winch.getInstance();
  private BooleanSubscriber winchSub;
  private boolean forwards;
  private BooleanPublisher winchPub;

  public Wrap(boolean forwards) {
    this.forwards = forwards;
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    winchPub = table.getBooleanTopic("winchOn").publish();
    winchSub = table.getBooleanTopic("winchOn").subscribe(true);

    addRequirements(winch);
  }

  @Override
  public void initialize() {
    winchPub.set(true);
  }

  @Override
  public void execute() {
    if (forwards) winch.extend();
    else winch.retract();
  }

  @Override
  public void end(boolean interrupted) {
    winch.stop();
  }

  @Override
  public boolean isFinished() {
    return !winchSub.get();
  }
}