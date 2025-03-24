// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.subsystems.Elevator;
 import frc.team2641.robot2025.Constants.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElev extends InstantCommand {

  private Elevator elev = Elevator.getInstance();
  private double setPoint;
  /** Creates a new SetElev. */
  public SetElev(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    

    this.setPoint = setPoint;

    addRequirements(elev);
  }

  public SetElev(Constants.ELEVNUM pos)
  {
    setPoint = 0;

    if( pos != null)
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

    addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elev.goTo(setPoint);
  }
}