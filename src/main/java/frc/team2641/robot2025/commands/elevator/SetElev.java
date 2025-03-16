// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.subsystems.elevator.Elevator;
import frc.team2641.robot2025.subsystems.elevator.ElevatorIO;
 import frc.team2641.robot2025.Constants.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElev extends Command {

  private ElevatorIO elev;
  private double setPoint;
  /** Creates a new SetElev. */
  public SetElev(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    elev = Elevator.getInstance();

    this.setPoint = setPoint;

    addRequirements(elev);
  }

  public SetElev(Constants.ELEVNUM pos){
    switch (pos) {
      case L1:
        setPoint = ElevatorPositions.L1;
        break;
      
        case L2:
        setPoint = ElevatorPositions.L2;
        break;

        case L3:
        setPoint = ElevatorPositions.L3;
        break;
      
        case L4:
        setPoint = ElevatorPositions.L4;
        break;

        case HP:
        setPoint = ElevatorPositions.HP;
        break;
    
      default:
      setPoint = 0;
        break;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elev.setAuto(true);
    elev.goTo(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elev.setAuto(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elev.atPosition() && elev.getAuto();
  }
}
