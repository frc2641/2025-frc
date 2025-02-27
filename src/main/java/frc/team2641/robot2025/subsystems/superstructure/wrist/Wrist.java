package frc.team2641.robot2025.subsystems.superstructure.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;

public class Wrist extends SubsystemBase {
  public static WristIO getInstance() {
    if (!Robot.isSimulation())
      return WristReal.getInstance();
    return WristSim.getInstance();
  }
}