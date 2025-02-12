package frc.team2641.robot2025.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.RobotContainer;

public class Intake extends SubsystemBase implements IntakeIO{
  private TalonFX intake1;
  private TalonFX intake2;
  
  private BooleanSubscriber spinSub;
  
  private static Intake instance;
  public static IntakeIO getInstance() {
    if(!Robot.isSimulation()){
    if (instance == null)
      instance = new Intake();
    return instance;
    }
    return IntakeIOSim.getInstance();
  }

  private Intake() {
    // TODO: Set ID
    intake1 = new TalonFX(Constants.CAN.intake1);
    intake2 = new TalonFX(Constants.CAN.intake2);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("reverseIntake").subscribe(false);
  }

  public void in() {
    intake1.set(Constants.MotorSpeeds.intakeSpeed);
    intake2.set(Constants.MotorSpeeds.intakeSpeed);
  }

  public void out() {
    intake1.set(-Constants.MotorSpeeds.intakeSpeed);
    intake2.set(-Constants.MotorSpeeds.intakeSpeed);
  }

  @Override
  public void spin() {
    if(spinSub.get()){
      out();
      
    }
    else{
      in();
    }
  }

  @Override
  public void stop() {
    intake1.stopMotor();
    intake2.stopMotor();
  }

  @Override
  public void periodic() {
  }


}