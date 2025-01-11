package frc.team2641.robot2025.commands;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Shooter;

public class Rev extends Command {
  private Shooter shooter;
  private int speed;
  IntegerPublisher speedPub;

  public Rev(int speed) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    speedPub = table.getIntegerTopic("shooterSpeed").publish();
    speedPub.set(0);

    shooter = Shooter.getInstance();
    this.speed = speed;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    speedPub.set(speed);
    if (speed == 1)
      shooter.amp();
    else if (speed == 2)
      shooter.speaker();
    else if (speed == 3)
      shooter.trap();
    else if (speed == 4)
      shooter.intake();
  }

  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}