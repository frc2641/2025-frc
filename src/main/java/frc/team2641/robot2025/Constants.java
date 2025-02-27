package frc.team2641.robot2025;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.team2641.robot2025.helpers.ArmPosition;
import swervelib.math.Matter;

public final class Constants {
  public static final double ROBOT_MASS = 47; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.25, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(6, 0, 0);
  }

  public static final class DriveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double MAX_SPEED = 10; // m/s
  }

  public static class OperatorConstants {
    public static final double LEFT_X_DEADBAND = 0.1;
    
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class CAN {
    public static final int elevator = 13;
    public static final int intake1 = 14;
    public static final int intake2 = 15;
    public static final int wrist = 13;
    public static final int climber = 17; // elevator motor ID, set to 3 for testing

    public static final int pdh = 20;
    public static final int ph = 21;
  }

  public static final class MotorSpeeds {
    public static final double elevatorSpeed = 0;
    public static final double climbSpeed = 0.25;
    public static final double wristSpeed = 1;
    public static final double intakeSpeed = 0;
  }

  public static final class IntakeGains {
    /** @see TO_DO is currently a random number to make stuff do stuff */
    public static final PIDConstants wristGains = new PIDConstants(0, 0, 0);
    public static final SlewRateLimiter wristRateLimiter = new SlewRateLimiter(6);
   /** @see TO_DO is currently a random number to make stuff do stuff */
    public static final PIDConstants elevatorGains = new PIDConstants(0, 0, 0);
    public static final SlewRateLimiter elevatorRateLimiter = new SlewRateLimiter(6);

    public static final PIDConstants climbGains = new PIDConstants(6, 0, 0);
    public static final SlewRateLimiter climbRateLimiter = new SlewRateLimiter(6);

    public static final double elevMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond); 
    public static final double elevMaxAcceleration = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final class ElevatorConstants
    {
      // public static final double kS = 0.02; //guessed
      // public static final double kG = 0.9; //guessed
      // public static final double kV = 3.8; //guessed
      // public static final double kA = 0.17; //guessed
      
      // public static final double kGearing = 1; //motor gear ratio
      // public static final double kCarraigeMass = -1; //TODO ! ! ! in 
      // public static final double kDrumRadius = Units.inchesToMeters(-1); // TODO !!!

      // public static final double maxHeight = Units.inchesToMeters(0);
      // public static final double minHeight = 0;

      public static final int kMotorPort = 0;
      public static final int kEncoderAChannel = 0;
      public static final int kEncoderBChannel = 1;
      public static final int kJoystickPort = 0;
    
      public static final double kElevatorKp = 5;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0;
    
      public static final double kElevatorkS = 0.0; // volts (V)
      public static final double kElevatorkG = 0.762; // volts (V)
      public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
      public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
    
      public static final double kElevatorGearing = 7.75;
      public static final double kElevatorDrumRadius = Units.inchesToMeters(1/2.0);
      public static final double kCarriageMass = 15.8757; // kg
    /**idk what that is*/
      public static final double kSetpointMeters = 0.75;

      // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
      public static final double kMinElevatorHeightMeters = 0.0;
      public static final double kMaxElevatorHeightMeters = 2.0574;
    
      // distance per pulse = (distance per revolution) / (pulses per revolution)
      //  = (Pi * D) / ppr
      public static final double kElevatorEncoderDistPerPulse =
          2.0 * Math.PI * kElevatorDrumRadius / 4096;
    }
  }

  public static final class ArmPositions {
    public static final ArmPosition L1 = new ArmPosition(-1, -1);
    public static final ArmPosition L2 = new ArmPosition(-1, -1);
    public static final ArmPosition L3 = new ArmPosition(-1, -1);
    public static final ArmPosition L4 = new ArmPosition(-1, -1);
    public static final ArmPosition humanPlayer = new ArmPosition(-1, -1);
    public static final ArmPosition processor = new ArmPosition(-1, -1);
    public static final ArmPosition algaeRemoval = new ArmPosition(-1, -1);
  } 
}
