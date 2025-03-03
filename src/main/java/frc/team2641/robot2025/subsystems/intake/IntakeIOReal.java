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
    if (instance == null) instance = new IntakeIOReal();
    return instance;
  }

  private IntakeIOReal() {
    leftIntake = new TalonFX(Constants.CAN.leftIntake);
    rightIntake = new TalonFX(Constants.CAN.rightIntake);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("reverseIntake").subscribe(false);
  }

  public void intake() {
    leftIntake.set(Constants.IntakeConstants.leftIntakeSpeed);
    rightIntake.set(-Constants.IntakeConstants.rightIntakeSpeed);
  }

  public void shoot() {
    leftIntake.set(-Constants.IntakeConstants.leftIntakeSpeed);
    rightIntake.set(Constants.IntakeConstants.rightIntakeSpeed);
  }

  public void spin() {
    if (spinSub.get()) shoot();
    else intake();
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