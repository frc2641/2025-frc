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
  private TalonFX motor;
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

  @Override
  public void spin() {
    if (spinSub.get())  
      motor.set(IntakeConstants.speed);
    else 
      motor.set(IntakeConstants.speed);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  private void configMotor() {
    motor = new TalonFX(CANConstants.rightIntake);
    
    TalonFXConfigurator configer = motor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // TODO numbers are untested
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 45;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    // config.CurrentLimits.StatorCurrentLimit = 60;

    configer.apply(config);
  }

  @Override
  public void periodic() {
    if (Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < IntakeConstants.stallV) {
      System.out.println("\n\n *** STALL DETECTED - INTAKE *** \n\n");
    }
  }

  public TalonFX getMotor() {
    return motor;
  }
}