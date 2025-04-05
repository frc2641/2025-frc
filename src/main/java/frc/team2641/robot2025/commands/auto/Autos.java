package frc.team2641.robot2025.commands.auto;

import java.util.ArrayList;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team2641.robot2025.Alerts;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.Constants.ELEVNUM;
import frc.team2641.robot2025.commands.elevator.SetElevator;
import frc.team2641.robot2025.commands.intake.RunIntake;
import frc.team2641.robot2025.commands.intake.RunOuttake;
import frc.team2641.robot2025.commands.intake.SuperSpin;
import frc.team2641.robot2025.subsystems.Elevator;
import frc.team2641.robot2025.subsystems.Intake;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class Autos {
	private static Pose2d startLeftCage;
	private static Pose2d startMidCage;
	private static Pose2d startRightCage;

	private static PathPlannerPath topFirstPath;
	private static PathPlannerPath middleFirstPath;
	private static PathPlannerPath bottomFirstPath;

	private static final SendableChooser<Autos.Mode> autoMode = new SendableChooser<Autos.Mode>();
	private static final SendableChooser<String> simpleAutoChooser = new SendableChooser<String>();
	private static final SendableChooser<ELEVNUM> simpleAutoHeight = new SendableChooser<ELEVNUM>();
	private static final SendableChooser<Pose2d> start = new SendableChooser<Pose2d>();
	private static final ArrayList<SendableChooser<Pose2d>> sources = new ArrayList<SendableChooser<Pose2d>>();
	private static final ArrayList<SendableChooser<Pose2d>> destinations = new ArrayList<SendableChooser<Pose2d>>();
	private static final ArrayList<SendableChooser<Constants.ELEVNUM>> levels = new ArrayList<SendableChooser<Constants.ELEVNUM>>();

	private static final Drivetrain drivetrain = Drivetrain.getInstance();

	public static void init() {
		NamedCommands.registerCommand("creep", new Creep());
    // NamedCommands.registerCommand("angleSource", new AutoAngle(4, true));
    NamedCommands.registerCommand("ElevL4", new SetElevator(ELEVNUM.L4));
    NamedCommands.registerCommand("ElevL3", new SetElevator(ELEVNUM.L3));
    NamedCommands.registerCommand("ElevL2", new SetElevator(ELEVNUM.L2));
    NamedCommands.registerCommand("ElevL1", new SetElevator(ELEVNUM.L1));
    NamedCommands.registerCommand("ElevHP", new SetElevator(ELEVNUM.HP));
    NamedCommands.registerCommand("ElevDown", new SetElevator(0));
    NamedCommands.registerCommand("spin", new SuperSpin());
    NamedCommands.registerCommand("Outtake", new RunOuttake());
		NamedCommands.registerCommand("PatientOuttake", Commands.race(
			new RunOuttake(),
			Commands.waitSeconds(2)
		));
    NamedCommands.registerCommand("Intake", new RunIntake());
    // NamedCommands.registerCommand("ChosenElev", Commands.runOnce(() -> new SetElevator(simpleAutoHeight.getSelected())));
    NamedCommands.registerCommand("ChosenElev", new SetElevator(simpleAutoHeight));

		autoMode.setDefaultOption("Simple", Autos.Mode.SIMPLE);
    autoMode.addOption("Build Your Own", Autos.Mode.BYO);
		autoMode.addOption("Creep", Autos.Mode.CREEP);
    autoMode.addOption("None", Autos.Mode.NONE);

		autoMode.onChange((Mode mode) -> checkValidAuto());
    
    SmartDashboard.putData("Auto Mode", autoMode);

		try {
			topFirstPath = PathPlannerPath.fromPathFile("Top First");
			middleFirstPath = PathPlannerPath.fromPathFile("Middle First");
			bottomFirstPath = PathPlannerPath.fromPathFile("Bottom First");
		} catch (Exception e) {
			System.err.println("Could not load paths");
			e.printStackTrace();
		}

		try {
			startLeftCage = topFirstPath.getStartingHolonomicPose().isPresent() ? topFirstPath.getStartingHolonomicPose().get() : new Pose2d(7.592,7.239, new Rotation2d(180 * Math.PI / 180));
			startMidCage = middleFirstPath.getStartingHolonomicPose().isPresent() ? middleFirstPath.getStartingHolonomicPose().get() : new Pose2d(7.592, 6.180, new Rotation2d(180 * Math.PI / 180));
			startRightCage = bottomFirstPath.getStartingHolonomicPose().isPresent() ? bottomFirstPath.getStartingHolonomicPose().get() : new Pose2d(7.592, 5.083, new Rotation2d(180 * Math.PI / 180));
		} catch (Exception e) {
			startLeftCage = new Pose2d(7.592,7.239, new Rotation2d(270 * Math.PI / 180));
			startMidCage = new Pose2d(7.592, 6.180, new Rotation2d(270 * Math.PI / 180));
			startRightCage = new Pose2d(7.592, 5.083, new Rotation2d(270 * Math.PI / 180));
			e.printStackTrace();
		}

		start.addOption("Left", startLeftCage);
		start.addOption("Middle", startMidCage);
		start.addOption("Right", startRightCage);
		start.addOption("None", null);
		if (!DriverStation.getLocation().isEmpty()) {
			switch (DriverStation.getLocation().getAsInt()) {
				case 1:
					start.setDefaultOption("Left", startLeftCage);
					drivetrain.resetOdometry(startLeftCage);
					break;
				case 2:
					start.setDefaultOption("Middle", startMidCage);
					drivetrain.resetOdometry(startMidCage);
					break;
				case 3:
					start.setDefaultOption("Right", startRightCage);
					drivetrain.resetOdometry(startRightCage);
					break;
			}
		}
		start.onChange((Pose2d pose) -> {
			if (pose != null) drivetrain.resetOdometry(pose);
			checkValidAuto();
		});

		sources.add(null);
		for (int i = 1; i < 3; i++) {
			sources.add(new SendableChooser<Pose2d>());
			initSource(sources.get(i));
		}

		for (int i = 0; i < 3; i++) {
			destinations.add(new SendableChooser<Pose2d>());
			initDestination(destinations.get(i));
		}

		for (int i = 0; i < 3; i++) {
			levels.add(new SendableChooser<Constants.ELEVNUM>());
			initLevel(levels.get(i));
		}

		SmartDashboard.putData("Starting Pose", start);

		// CYCLE 1
		// No source for Cycle 1
		SmartDashboard.putData("Cycle 1 Node", destinations.get(0));
		SmartDashboard.putData("Cycle 1 Level", levels.get(0));

		// CYCLE 2
		SmartDashboard.putData("Cycle 2 Source", sources.get(1));
		SmartDashboard.putData("Cycle 2 Node", destinations.get(1));
		SmartDashboard.putData("Cycle 2 Level", levels.get(1));

		// CYCLE 3
		SmartDashboard.putData("Cycle 3 Source", sources.get(2));
		SmartDashboard.putData("Cycle 3 Node", destinations.get(2));
		SmartDashboard.putData("Cycle 3 Level", levels.get(2));

		// SIMPLE AUTO
		simpleAutoChooser.setDefaultOption("Middle Cage to J Branch", "Middle Cage to J Branch");
    simpleAutoChooser.addOption("Middle Cage to K Branch", "Middle Cage to K Branch");
    simpleAutoChooser.addOption("Left Cage to J Branch", "Left Cage to J Branch");
    simpleAutoChooser.addOption("Left Cage to K Branch", "Left Cage to K Branch");
    simpleAutoChooser.addOption("Right Cage to J Branch", "Right Cage to J Branch");
    simpleAutoChooser.addOption("Right Cage to G Branch", "Right Cage to G Branch");
    simpleAutoChooser.addOption("Right Cage to K Branch", "Right Cage to K Branch");
		simpleAutoChooser.addOption("Center to G Branch", "Center to G Branch");
    simpleAutoChooser.addOption("Just Go Forward", "Straight");
    simpleAutoChooser.addOption("None", null);

		simpleAutoChooser.onChange((String auto) -> checkValidAuto());

    SmartDashboard.putData("Simple Auto Path", simpleAutoChooser);

    simpleAutoHeight.addOption("L1", ELEVNUM.L1);
    simpleAutoHeight.addOption("L2", ELEVNUM.L2);
    simpleAutoHeight.addOption("L3", ELEVNUM.L3);
    simpleAutoHeight.setDefaultOption("L4", ELEVNUM.L4);
		simpleAutoHeight.addOption("None", null);

		simpleAutoChooser.onChange((String auto) -> checkValidAuto());

    SmartDashboard.putData("Simple Auto Level", simpleAutoHeight);
	}

	private static void initSource(SendableChooser<Pose2d> sc) {
		sc.setDefaultOption("Left", HumanPlayerPoses.left);
		sc.addOption("Right", HumanPlayerPoses.right);
		sc.addOption("None", null);

		sc.onChange((Pose2d pose) -> checkValidAuto());
	}

	private static void initDestination(SendableChooser<Pose2d> sc) {
		sc.addOption("None", null);
		sc.addOption("A", ReefPoses.reefA);
		sc.addOption("B", ReefPoses.reefB);
		sc.addOption("C", ReefPoses.reefC);
		sc.addOption("D", ReefPoses.reefD);
		sc.addOption("E", ReefPoses.reefE);
		sc.addOption("F", ReefPoses.reefF);
		sc.addOption("G", ReefPoses.reefG);
		sc.addOption("H", ReefPoses.reefH);
		sc.addOption("I", ReefPoses.reefI);
		sc.setDefaultOption("J", ReefPoses.reefJ);
		sc.addOption("K", ReefPoses.reefK);
		sc.addOption("L", ReefPoses.reefL);

		sc.onChange((Pose2d pose) -> checkValidAuto());
	}

	private static void initLevel(SendableChooser<Constants.ELEVNUM> sc) {
		sc.addOption("L1", ELEVNUM.L1);
		sc.addOption("L2", ELEVNUM.L2);
		sc.addOption("L3", ELEVNUM.L3);
		sc.setDefaultOption("L4", ELEVNUM.L4);
		sc.addOption("None", null);

		sc.onChange((Constants.ELEVNUM level) -> checkValidAuto());
	}

	public static boolean checkValidAuto() {
		Robot.getInstance().robotContainer.updateAlerts();
		boolean result = true;

		if (start.getSelected() == null) {
			result = false;
			Alerts.NoStartPose.set(true);
		} else Alerts.NoStartPose.set(false);

		switch (autoMode.getSelected()) {
			case BYO:
				if (destinations.get(0).getSelected() == null || levels.get(0).getSelected() == null) result = false;
				if (sources.get(1).getSelected() == null || destinations.get(1).getSelected() == null || levels.get(1).getSelected() == null) result = false;
				if (sources.get(2).getSelected() == null || destinations.get(2).getSelected() == null || levels.get(2).getSelected() == null) result = false;
				break;
			case SIMPLE:
				if (simpleAutoChooser.getSelected() == null || simpleAutoHeight.getSelected() == null) result = false;
				break;
			case CREEP:
				break;
			default:
				Alerts.InvalidAuto.set(false);
				Alerts.ValidAuto.set(false);
				Alerts.AutoOff.set(true);
				return true;
		}

		if (!result) {
			Alerts.InvalidAuto.set(true);
			Alerts.ValidAuto.set(false);
			Alerts.AutoOff.set(false);
		} else {
			Alerts.InvalidAuto.set(false);
			Alerts.ValidAuto.set(true);
			Alerts.AutoOff.set(false);
		}

		return result;
	}

	public static Command getAutoCommand() {
		drivetrain.resetOdometry(start.getSelected());

		ArrayList<Command> sequence = new ArrayList<>();

		if (checkValidAuto()) {
			// CYCLE 1
			// Cycle 1 does not have a source

			if (destinations.get(0).getSelected() != null) {
				sequence.add(drivetrain.driveToPose(destinations.get(0).getSelected()));
			}

			if (levels.get(0).getSelected() != null) {
				sequence.add(new SetElevator(levels.get(0)));
				sequence.add(new RunOuttake().withTimeout(0.5));
			}

			// CYCLE 2
			if (sources.get(1).getSelected() != null) {
				sequence.add(drivetrain.driveToPose(sources.get(1).getSelected()));
				sequence.add(new RunIntake().withTimeout(1));
			}

			if (destinations.get(1).getSelected() != null) {
				sequence.add(drivetrain.driveToPose(destinations.get(1).getSelected()));
			}

			if (levels.get(1).getSelected() != null) {
				sequence.add(new SetElevator(levels.get(1)));
				sequence.add(new RunOuttake().withTimeout(0.5));
			}

			// CYCLE 3
			if (sources.get(2).getSelected() != null) {
				sequence.add(drivetrain.driveToPose(sources.get(2).getSelected()));
				sequence.add(new RunIntake().withTimeout(1));
			}

			if (destinations.get(2).getSelected() != null) {
				sequence.add(drivetrain.driveToPose(destinations.get(2).getSelected()));
			}

			if (levels.get(2).getSelected() != null) {
				sequence.add(new SetElevator(levels.get(2)));
				sequence.add(new RunOuttake().withTimeout(0.5));
			}

			// END
			sequence.add(new SetElevator(0));
		} else {
			sequence.add(new Creep());
		}

		Command result = Commands.sequence(sequence.toArray(new Command[0]));

		// REQUIREMENTS
		result.addRequirements(Intake.getInstance());
		result.addRequirements(Elevator.getInstance());
		result.addRequirements(drivetrain);

		return result;
	}

	public static Command getSimpleAuto() {
		return drivetrain.getAutonomousCommand(simpleAutoChooser.getSelected());
	}

	public static Mode getMode() {
		return autoMode.getSelected();
	}

	public static class HumanPlayerPoses {	
		public static final Pose2d left = new Pose2d(1.31, 7.181, new Rotation2d(305 * Math.PI / 180));
		public static final Pose2d right = new Pose2d(1.337, 0.841, new Rotation2d(55 * Math.PI / 180));
	}

	public static class ReefPoses {
		public static final Pose2d reefA = new Pose2d(3.214, 4.381, new Rotation2d(-90 * Math.PI / 180));
		public static final Pose2d reefB = new Pose2d(3.21, 4.140, new Rotation2d(-90 * Math.PI / 180));
		public static final Pose2d reefC = new Pose2d(3.897, 2.822, new Rotation2d(60 * Math.PI / 180));
		public static final Pose2d reefD = new Pose2d(4.185, 2.659, new Rotation2d(-30 * Math.PI / 180));
		public static final Pose2d reefE = new Pose2d(5.196, 2.890, new Rotation2d(30 * Math.PI / 180));
		public static final Pose2d reefF = new Pose2d(5.513, 3.053, new Rotation2d(30 * Math.PI / 180));
		public static final Pose2d reefG = new Pose2d(5.821, 4.112, new Rotation2d(90 * Math.PI / 180));
		public static final Pose2d reefH = new Pose2d(5.821, 4.400, new Rotation2d(90 * Math.PI / 180));
		public static final Pose2d reefI = new Pose2d(5.071, 5.218, new Rotation2d(150 * Math.PI / 180));
		public static final Pose2d reefJ = new Pose2d(4.782, 5.391, new Rotation2d(150 * Math.PI / 180));
		public static final Pose2d reefK = new Pose2d(3.791, 5.151, new Rotation2d(-150 * Math.PI / 180));
		public static final Pose2d reefL = new Pose2d(3.473, 4.958, new Rotation2d(-150 * Math.PI / 180));
	}

	public static enum Mode { SIMPLE, BYO, CREEP, NONE }
}