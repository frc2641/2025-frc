package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Limelight;
import frc.team2641.robot2025.subsystems.Drivetrain;

public class LimelightTracking extends Command {
  private final Drivetrain swerveSubsystem;

  private final PIDController controllerX;
  // private final PIDController controllerY;
  private final PIDController controllerAngle;

  public LimelightTracking() {
    this.swerveSubsystem = Drivetrain.getInstance();

    controllerX = new PIDController(3, 1, 0);
    controllerX.setTolerance(0.025);
    controllerX.setSetpoint(0.0);

    // controllerY = new PIDController(1.0, 0.0, 0.0);
    // controllerY.setTolerance(0.25);
    // controllerY.setSetpoint(0.0);

    controllerAngle = new PIDController(2.9, 0, 0);
    controllerAngle.setTolerance(0.01);
    controllerAngle.setSetpoint(0.0);

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command. Called once when the command is
   * initially scheduled.
   */
  @Override
  public void initialize() {
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled.
   * (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    double tx = Limelight.getBotPose3d_TargetSpace("").getX();
    // double ty = Limelight.getBotPose3d_TargetSpace("").getY();
    double tangle = Limelight.getBotPose3d_TargetSpace("").getRotation().getY();

    // double translationY = MathUtil.clamp(controllerY.calculate(ty, 0.0), -0.5, 0.5);
    double translationX = MathUtil.clamp(controllerX.calculate(tx, 0.0), -1, 1);
    double rotation = MathUtil.clamp(controllerAngle.calculate(tangle, 0.0), -1, 1);

    swerveSubsystem.drive(new Translation2d(0, translationX), -rotation, false);
    // swerveSubsystem.drive(new Translation2d(0, -translationY), rotation, false);
    // System.out.println("ty: " + ty + " tangle: " + tangle);
    SmartDashboard.putNumber("translationX", translationX);
    SmartDashboard.putNumber("rotation", rotation);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes --
   * indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p>
   * <p>
   * Returning false will result in the command never ending automatically. It may
   * still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return
   * true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such
   * an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return Limelight.getFiducialID("") != -1 && controllerX.atSetpoint() && controllerAngle.atSetpoint();
    // return false;
  }

  /**
   * The action to take when the command ends. Called when either the command
   * finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }
}
