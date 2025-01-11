package frc.team2641.robot2025.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Drivetrain;
import frc.team2641.robot2025.subsystems.Indexer;

public class Feed extends Command {
  private Indexer indexer;
  private Drivetrain drivetrain;
  IntegerSubscriber speedSub;

  public Feed() {
    drivetrain = Drivetrain.getInstance();
    indexer = Indexer.getInstance();
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    speedSub = table.getIntegerTopic("shooterSpeed").subscribe(0);

    if (speedSub.get() == 1) {
      indexer.amp();
      Timer.delay(0.5);
      drivetrain.drive(new Translation2d(50, 0), 0, false);
      Timer.delay(0.5);
      drivetrain.drive(new Translation2d(-50, 0), 0, false);
      Timer.delay(0.5);
      drivetrain.drive(new Translation2d(0, 0), 0, false);
    } else if (speedSub.get() == 2)
      indexer.speaker();
    else if (speedSub.get() == 3)
      indexer.trap();
    else if (speedSub.get() == 4)
      indexer.intake();
    else
      indexer.stop();
  }

  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}