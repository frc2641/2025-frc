package frc.team2641.robot2025.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.climber.ClimberReal;

public class Climb extends Command {
  private ClimberReal climber = ClimberReal.getInstance();
  private boolean forwards;
  private BooleanPublisher winchPub;
  private BooleanSubscriber winchSub;

  public Climb(boolean forwards) {
    this.forwards = forwards;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    winchPub = table.getBooleanTopic("winchOn").publish();
    winchSub = table.getBooleanTopic("winchOn").subscribe(true);

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    winchPub.set(false);
  }

  @Override
  public void execute() {
    if (forwards) climber.extend();
    else climber.retract();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return winchSub.get();
  }
}