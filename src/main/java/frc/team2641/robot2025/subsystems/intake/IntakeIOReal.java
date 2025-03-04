package frc.team2641.robot2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private BooleanSubscriber spinSub;
  
  private static IntakeIOReal instance;
  
  public static IntakeIO getInstance() {
    if (instance == null) instance = new IntakeIOReal();
    return instance;
  }

  private IntakeIOReal() {
    configMotor();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("reverseIntake").subscribe(false);
  }

  public void intake() {
    rightMotor.set(-IntakeConstants.rightSpeed);
    leftMotor.set(IntakeConstants.leftSpeed);
  }

  public void shoot() {
    rightMotor.set(IntakeConstants.rightSpeed);
    leftMotor.set(-IntakeConstants.leftSpeed);
  }

  public void spin() {
    if (spinSub.get()) shoot();
    else intake();
  }

  @Override
  public void stop() {
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }

  private void configMotor() {
    leftMotor = new TalonFX(CANConstants.leftIntake);
    rightMotor = new TalonFX(CANConstants.rightIntake);
    
    TalonFXConfigurator leftConfiger = leftMotor.getConfigurator();
    TalonFXConfigurator rightConfiger = leftMotor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    leftConfiger.apply(config);
    rightConfiger.apply(config);
  }

  @Override
  public void periodic() {
    if ((Math.abs(leftMotor.getVelocity().getValue().baseUnitMagnitude()) < IntakeConstants.stallV) && (leftMotor.getTorqueCurrent().getValue().baseUnitMagnitude() > IntakeConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - LEFT INTAKE *** \n\n");
    }

    if ((Math.abs(rightMotor.getVelocity().getValue().baseUnitMagnitude()) < IntakeConstants.stallV) && (rightMotor.getTorqueCurrent().getValue().baseUnitMagnitude() > IntakeConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - RIGHT INTAKE *** \n\n");
    }
  }

  public TalonFX getMotor() {
    return rightMotor;
  }
}