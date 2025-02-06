package frc.team2641.robot2025.commands.shifts;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class ReverseIntake extends Command {
  BooleanPublisher spinningPub;

  public ReverseIntake() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");

    spinningPub = table.getBooleanTopic("reverseIntake").publish();
    spinningPub.set(false);
  }

  @Override
  public void initialize() {
    spinningPub.set(true);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    spinningPub.set(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
