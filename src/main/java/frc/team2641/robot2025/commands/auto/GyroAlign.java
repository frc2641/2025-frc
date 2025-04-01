package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants.AutoAngleNum;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class GyroAlign extends Command {
	private Drivetrain drivetrain = Drivetrain.getInstance();

  private BooleanPublisher gyroAlignPub;
  private DoublePublisher gyroAlignVPub;

	private AutoAngleNum target;

	public GyroAlign(AutoAngleNum target, boolean isAutonomous) {
		if (target == null)
			end(false);
		else	
			this.target = target;
			
		NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
		
		gyroAlignPub = table.getBooleanTopic("gyroAlign").publish();
		gyroAlignPub.set(false);

		gyroAlignVPub = table.getDoubleTopic("angularVelocity").publish();
		gyroAlignVPub.set(0);
	}

	public void initialize() {
		gyroAlignPub.set(true);
	}

	public void execute() {
		if (target.get() < 0) {
			if (drivetrain.getHeading().getDegrees() > target.get() && drivetrain.getHeading().getDegrees() < target.get()) {
				if (drivetrain.getHeading().getDegrees() < target.get()+20 && drivetrain.getHeading().getDegrees() > 0) {
					gyroAlignVPub.set(-0.4);
				} else {
					gyroAlignVPub.set(-0.65);
				}
			} else {
				if (drivetrain.getHeading().getDegrees() > target.get()-20 && drivetrain.getHeading().getDegrees() < 0) {
					gyroAlignVPub.set(0.4);
				} else {
					gyroAlignVPub.set(0.65);
				}
			}
		} else {
			if (drivetrain.getHeading().getDegrees() < target.get() && drivetrain.getHeading().getDegrees() > target.get()) {
				if (drivetrain.getHeading().getDegrees() > target.get()-20 && drivetrain.getHeading().getDegrees() < 0) {
					gyroAlignVPub.set(0.4);
				} else {
					gyroAlignVPub.set(0.65);
				}
			} else {
				if (drivetrain.getHeading().getDegrees() < target.get()+20 && drivetrain.getHeading().getDegrees() > 0) {
					gyroAlignVPub.set(-0.4);
				} else {
					gyroAlignVPub.set(-0.65);
				}
			}
		}
	}

	public void end(boolean interrupted) {
		gyroAlignPub.set(false);
		gyroAlignVPub.set(0);
	}

	public boolean isFinished() {
		return (drivetrain.getHeading().getDegrees() < target.get()+1 && drivetrain.getHeading().getDegrees() > target.get()-1);
	}
}