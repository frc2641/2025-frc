package frc.team2641.robot2025;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.team2641.robot2025.helpers.ArmPos;
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
    
    public static final int elevMotor = -1;
    public static final int intakeMotor1 = -1;
    public static final int intakeMotor2 = -1;
    public static final int wrist = -1;
    public static final int climb = -1;

    public static final int pdh = 20;
    public static final int ph = 21;
  }

  /** @see IMPORTANT needs to have speeds set */
  public static final class MotorSpeeds{
    public static final double elevatorSpeed = 0;
    public static final double climbSpeed = 0;
    public static final double intakeSpeed = 0;
  }
  public static final class IntakeGains{
    public static final PIDConstants wristGains = new PIDConstants(0, 0, 0);
    public static final SlewRateLimiter wristRateLimiter = new SlewRateLimiter(6);
   
    public static final PIDConstants elevatorGains = new PIDConstants(0, 0, 0);
    public static final SlewRateLimiter elevatorRateLimiter = new SlewRateLimiter(6);
    
  }
  public static final class ClimberPositions{
    public static final double extended = 0.0;
    public static final double start = 0.0;

  }

  public static final class ArmPositions{
    public static final ArmPos L1 = new ArmPos(-1, -1);
    public static final ArmPos L2 = new ArmPos(-1, -1);
    public static final ArmPos L3 = new ArmPos(-1, -1);
    public static final ArmPos L4 = new ArmPos(-1, -1);
    public static final ArmPos humanPlayer = new ArmPos(-1, -1);
    public static final ArmPos processor = new ArmPos(-1, -1);
  }

  
}
