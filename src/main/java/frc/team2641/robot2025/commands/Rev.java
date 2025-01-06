// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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

  /** Creates a new Flywheel. */
  public Rev(int speed) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    speedPub = table.getIntegerTopic("shooterSpeed").publish();
    speedPub.set(0);

    shooter = Shooter.getInstance();
    this.speed = speed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
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

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}