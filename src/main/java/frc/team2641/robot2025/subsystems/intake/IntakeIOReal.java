package frc.team2641.robot2025.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team2641.robot2025.Constants;

public class IntakeIOReal implements IntakeIO {
  private TalonFX leftIntake;
  private TalonFX rightIntake;
  private BooleanSubscriber spinSub;
  
  private static IntakeIOReal instance;
  
  public static IntakeIO getInstance() {
    if (instance == null)
      instance = new IntakeIOReal();
    return instance;
  }

  private IntakeIOReal() {
    // TODO: Set ID
    leftIntake = new TalonFX(Constants.CANConstants.leftIntake);
    rightIntake = new TalonFX(Constants.CANConstants.rightIntake);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("reverseIntake").subscribe(false);
  }

  public void intake() {
    leftIntake.set(-Constants.IntakeConstants.leftIntake);
    rightIntake.set(Constants.IntakeConstants.rightIntake);
  }

  public void shoot() {
    leftIntake.set(Constants.IntakeConstants.leftIntake);
    rightIntake.set(-Constants.IntakeConstants.leftIntake);
  }

  public void spin() {
    if(spinSub.get())
      shoot();
    else 
      intake();
  }

  @Override
  public void stop() {
    leftIntake.stopMotor();
    rightIntake.stopMotor();
  }

  @Override
  public void periodic() {
  }
}