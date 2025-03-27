package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor;
  private TalonFX outtakeMotor;
  
  private static Intake instance;
  
  public static Intake getInstance() {
    if (instance == null) instance = new Intake();
    return instance;
  }

  private Intake() {
    configMotor();

  }

  public void stop() {
    intakeMotor.stopMotor();
    outtakeMotor.stopMotor();
  }

  private void configMotor() {
    intakeMotor = new TalonFX(CANConstants.intake);
    outtakeMotor = new TalonFX(CANConstants.outtake);

    
    TalonFXConfigurator configer1 = intakeMotor.getConfigurator();
    TalonFXConfigurator configer2 = outtakeMotor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    TalonFXConfiguration outConfig = new TalonFXConfiguration();
    outConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    outConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        // TODO numbers are untested
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 45;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    // config.CurrentLimits.StatorCurrentLimit = 60;

    configer1.apply(config);
    configer2.apply(outConfig);

  }

  public void periodic() {
    if (Robot.isReal() && Math.abs(intakeMotor.getVelocity().getValue().baseUnitMagnitude()) < IntakeConstants.stallV) {
      // System.out.println("\n\n *** STALL DETECTED - INTAKE *** \n\n");
    }
  }

  public TalonFX getMotor() {
    return intakeMotor;
  }

  public void firstSpin() {
    intakeMotor.set(IntakeConstants.speedIn1);
    outtakeMotor.set(IntakeConstants.speedIn2);
  }

  public void secondSpin() {
    outtakeMotor.set(IntakeConstants.speedOut);
    intakeMotor.set(IntakeConstants.speedOut);
  }

  public void superSpin(){
    outtakeMotor.set(IntakeConstants.superSpeed);
    intakeMotor.set(IntakeConstants.superSpeed);
  }
}