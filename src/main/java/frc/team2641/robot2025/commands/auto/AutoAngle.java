package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class AutoAngle extends Command {
	private Drivetrain drivetrain = Drivetrain.getInstance();

	private int targetAngle;
	private int oppositeAngle;
	private int element;
	private boolean isAutonomous;

	private BooleanPublisher alignmentPub;
	private DoublePublisher angularVelocityPub;
	private IntegerPublisher stagePub;
	private IntegerSubscriber stageSub;

	public AutoAngle(int element, boolean isAutonomous) {

		this.element = element;
		this.isAutonomous = isAutonomous;

		NetworkTable table = NetworkTableInstance.getDefault().getTable("state");

		stagePub = table.getIntegerTopic("stageAngle").publish();
		stagePub.set(0);
		stageSub = table.getIntegerTopic("stageAngle").subscribe(0);

		alignmentPub = table.getBooleanTopic("autoAlign").publish();
		alignmentPub.set(false);

		angularVelocityPub = table.getDoubleTopic("angularVelocity").publish();
		angularVelocityPub.set(0);
	}

	public void initialize() {
		alignmentPub.set(true);

		if (element == 1) {
			// Amp
			targetAngle = -90;
			oppositeAngle = 90;
		} else if (element == 2) {
			// Speaker
			targetAngle = 0;
			oppositeAngle = 180;
		} else if (element == 3) {
			// Stage
			if (stageSub.get() < 3)
				stagePub.set(stageSub.get()+1);
			else
				stagePub.set(1);

			if (stageSub.get() == 1) {
				targetAngle = -120;
				oppositeAngle = 60;
			} else if (stageSub.get() == 2) {
				targetAngle = 0;
				oppositeAngle = -180;
			} else if (stageSub.get() == 3) {
				targetAngle = 120;
				oppositeAngle = -60;
			}
		} else if (element == 4) {
			// Source
			targetAngle = 120;
			oppositeAngle = -60;
		}

		if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
			if (element != 3) {
				targetAngle *= -1;
				oppositeAngle *= -1;
			}
		}
	}
	
	public void execute() {
		if (targetAngle < 0) {
			if (drivetrain.getHeading().getDegrees() > targetAngle && drivetrain.getHeading().getDegrees() < oppositeAngle) {
				if (drivetrain.getHeading().getDegrees() < targetAngle+20 && drivetrain.getHeading().getDegrees() > 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.4, true);
					else
						angularVelocityPub.set(-0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.65, true);
					else
						angularVelocityPub.set(-0.65);
				}
			} else {
				if (drivetrain.getHeading().getDegrees() > targetAngle-20 && drivetrain.getHeading().getDegrees() < 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.4, true);
					else
						angularVelocityPub.set(0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.65, true);
					else
						angularVelocityPub.set(0.65);
				}
			}
		} else {
			if (drivetrain.getHeading().getDegrees() < targetAngle && drivetrain.getHeading().getDegrees() > oppositeAngle) {
				if (drivetrain.getHeading().getDegrees() > targetAngle-20 && drivetrain.getHeading().getDegrees() < 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.4, true);
					else
						angularVelocityPub.set(0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.65, true);
					else
						angularVelocityPub.set(0.65);
				}
			} else {
				if (drivetrain.getHeading().getDegrees() < targetAngle+20 && drivetrain.getHeading().getDegrees() > 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.4, true);
					else
						angularVelocityPub.set(-0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.65, true);
					else
						angularVelocityPub.set(-0.65);
				}
			}
		}
	}

	public void end(boolean interrupted) {
		alignmentPub.set(false);
		if (isAutonomous)
			drivetrain.drive(new Translation2d(0, 0), 0, false);
		else
			angularVelocityPub.set(0);
	}

	public boolean isFinished() {
		return (drivetrain.getHeading().getDegrees()<(targetAngle+1) && drivetrain.getHeading().getDegrees()>(targetAngle-1));
	}
}
