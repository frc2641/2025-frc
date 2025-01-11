package frc.team2641.robot2025.commands.shifts;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotRelative extends Command {
  BooleanPublisher robotPub;

  public RobotRelative() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");

    robotPub = table.getBooleanTopic("robotRelative").publish();
    robotPub.set(false);
  }

  @Override
  public void initialize() {
    robotPub.set(true);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    robotPub.set(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
