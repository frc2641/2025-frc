package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Elevator extends SubsystemBase {
    private static Elevator instance;
    private TalonFX elevMotor1;
    private TalonFX elevMotor2;
    public static Elevator getInstance() {
      if (instance == null)
        instance = new Elevator();
      return instance;
    }   

    private Elevator()
    {
      // Needs ID set
      elevMotor1 = new TalonFX(-1);
      elevMotor2 = new TalonFX(-1);

    }
    public void up() {
      elevMotor1.set(Constants.MotorSpeeds.elevatorSpeed);
      elevMotor2.set(Constants.MotorSpeeds.elevatorSpeed);

    }
  
    public void down() {
      elevMotor1.set(-Constants.MotorSpeeds.elevatorSpeed);
      elevMotor2.set(-Constants.MotorSpeeds.elevatorSpeed);
    }
  
    public void stop() {
      elevMotor1.stopMotor();
      elevMotor2.stopMotor();
    }
/** 
 *@see 0 = ground state
 *@see 1 = L1
 *@see 2 = L2
 *@see 3 = L3
 *@see 4 = L4
 *@see 5 = human player
 *@see 6 = processor
 */
    public void setPos(int stage){
      if(stage<0){
        if(stage == -1)
          up();
        else if(stage ==-2)
          down();
      }
      double goal = 0;
      switch (stage) {
        case 1:

        break;

        case 2:
      
        break;

        case 3:

        break;

        case 4:

        break;

        case 5:

        break;

        case 6:

        break;

        default:

        break;
      }

      while(Math.abs(elevMotor1.getPosition().getValueAsDouble()-goal) < 0.05)
      {
          if(elevMotor1.getPosition().getValueAsDouble()-goal<0)
            up();
          if(elevMotor1.getPosition().getValueAsDouble()-goal>0)
            down();
      }
    }
  
  @Override
  public void periodic() {
  }
}
