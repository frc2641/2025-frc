package frc.team2641.robot2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;

public class Intake extends SubsystemBase {
  
  public static IntakeIO getInstance() {
    if(!Robot.isSimulation()) 
      return IntakeReal.getInstance();
    
    return IntakeIOSim.getInstance();
  }

  
}