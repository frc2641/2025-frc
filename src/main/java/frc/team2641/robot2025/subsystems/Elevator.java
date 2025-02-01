package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
      configMotors();
    }
    public void up() {
      set(Constants.MotorSpeeds.elevatorSpeed);

    }
  
    public void down() {
      set(-Constants.MotorSpeeds.elevatorSpeed);
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

      double goal = 0.0;
      switch (stage) {
        case 1:
        goal = Constants.ElevPositions.L1;
        break;

        case 2:
        goal = Constants.ElevPositions.L2;
        break;

        case 3:
        goal = Constants.ElevPositions.L3;
        break;

        case 4:
        goal = Constants.ElevPositions.L4;
        break;

        case 5:
        goal = Constants.ElevPositions.humanPlayer;
        break;

        case 6:
        goal = Constants.ElevPositions.processor;
        break;

        default:
        goal = Constants.ElevPositions.L1;
        break;
      }

      setPos(goal);
    }

    public void set(double value) {
      elevMotor1.set(Constants.IntakeGains.elevatorRateLimiter.calculate(value * Constants.IntakeGains.elevatorGains.kPeakOutput));
      elevMotor2.set(Constants.IntakeGains.elevatorRateLimiter.calculate(value * Constants.IntakeGains.elevatorGains.kPeakOutput));

    }

  public void setPos(double pos) {
    elevMotor1.set(ControlMode.Position, pos);
  }
  
  @Override
  public void periodic() {
  }

  private void configMotors(){
    

      
    elevMotor1.configAllowableClosedloopError(0, Constants.IntakeGains.elevatorGains.kAllowableError, 30);

    elevMotor1.config_kP(0, Constants.IntakeGains.elevatorGains.kP, 30);
    elevMotor1.config_kI(0, Constants.IntakeGains.elevatorGains.kI, 30);
    elevMotor1.config_kD(0, Constants.IntakeGains.elevatorGains.kD, 30);
    elevMotor1.config_kF(0, Constants.IntakeGains.elevatorGains.kF, 30);


    
    elevMotor2.configAllowableClosedloopError(0, Constants.IntakeGains.elevatorGains.kAllowableError, 30);

    elevMotor2.config_kP(0, Constants.IntakeGains.elevatorGains.kP, 30);
    elevMotor2.config_kI(0, Constants.IntakeGains.elevatorGains.kI, 30);
    elevMotor2.config_kD(0, Constants.IntakeGains.elevatorGains.kD, 30);
    elevMotor2.config_kF(0, Constants.IntakeGains.elevatorGains.kF, 30);

  }
}
