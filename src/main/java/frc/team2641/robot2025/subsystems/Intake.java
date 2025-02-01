package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;

    private TalonFX intake1;
    private TalonFX intake2;
    
    private BooleanSubscriber spinSub;

  public static Intake getInstance() {
    if (instance == null)
      instance = new Intake();
    return instance;
  }


  private Intake() {
    // Needs ID set
    intake1 = new TalonFX(Constants.CAN.intakeMotor1);
    intake2 = new TalonFX(Constants.CAN.intakeMotor2);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("intakeSpinningOut").subscribe(false);
  }

  public void in(){
    intake1.set(Constants.MotorSpeeds.intakeSpeed);
    intake2.set(Constants.MotorSpeeds.intakeSpeed);
  }

  public void out()
  {
    intake1.set(-Constants.MotorSpeeds.intakeSpeed);
    intake2.set(-Constants.MotorSpeeds.intakeSpeed);
  }
  public void spin(){
    if(spinSub.get())
      out();
    else
      in();
  }

  public void stop() {
    intake1.stopMotor();
    intake2.stopMotor();
  }

  @Override
  public void periodic() {
  }
}