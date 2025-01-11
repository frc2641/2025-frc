package frc.team2641.robot2025.commands.shifts;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class SniperMode extends Command {
  BooleanPublisher sniperPub;

  public SniperMode() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");

    sniperPub = table.getBooleanTopic("sniperMode").publish();
    sniperPub.set(false);
  }

  @Override
  public void initialize() {
    sniperPub.set(true);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    sniperPub.set(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
