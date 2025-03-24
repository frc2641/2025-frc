package frc.team2641.robot2025.commands.auto;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.ELEVNUM;
import frc.team2641.robot2025.commands.Wait;
import frc.team2641.robot2025.commands.elevator.SetElev;
import frc.team2641.robot2025.commands.intake.RunIntake;
import frc.team2641.robot2025.commands.intake.RunOuttake;
import frc.team2641.robot2025.subsystems.elevator.Elevator;
import frc.team2641.robot2025.subsystems.intake.Intake;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class Autos {
    
    
    
    public static Pose2d startLeftCage;
    public static Pose2d startMidCage;
    public static Pose2d startRightCage;

    private static final Drivetrain drive = Drivetrain.getInstance();
    
    
    private static void init(){
        try {
            startLeftCage =  PathPlannerPath.fromPathFile("Top First").getStartingHolonomicPose().isPresent() ? PathPlannerPath.fromPathFile("Top First").getStartingHolonomicPose().get() : new Pose2d(7.592,7.239, new Rotation2d(180 * Math.PI / 180));
            startMidCage =  PathPlannerPath.fromPathFile("Middle First").getStartingHolonomicPose().isPresent() ? PathPlannerPath.fromPathFile("Middle First").getStartingHolonomicPose().get() : new Pose2d(7.592, 6.180, new Rotation2d(180 * Math.PI / 180));
            startRightCage =  PathPlannerPath.fromPathFile("Bottom First").getStartingHolonomicPose().isPresent() ? PathPlannerPath.fromPathFile("Bottom First").getStartingHolonomicPose().get() : new Pose2d(7.592, 5.083, new Rotation2d(180 * Math.PI / 180));
            
        } catch (Exception e) {
            startLeftCage = new Pose2d(7.592,7.239, new Rotation2d(180 * Math.PI / 180));
            startMidCage = new Pose2d(7.592, 6.180, new Rotation2d(180 * Math.PI / 180));
            startRightCage = new Pose2d(7.592, 5.083, new Rotation2d(180 * Math.PI / 180));
            e.printStackTrace();
        }
    }

    public static final Pose2d humanPlayerT = new Pose2d(1.472, 7.210, new Rotation2d(35 * Math.PI / 180));
    public static final Pose2d humanPlayerB = new Pose2d(1.472, 0.831, new Rotation2d(145 * Math.PI / 180));
     
    public static final Pose2d reefA = new Pose2d(3.156, 3.938, new Rotation2d(90 * Math.PI / 180));
    public static final Pose2d reefB = new Pose2d(3.156, 3.621, new Rotation2d(90 * Math.PI / 180));
    public static final Pose2d reefC = new Pose2d(3.897, 2.822, new Rotation2d(150 * Math.PI / 180));
    public static final Pose2d reefD = new Pose2d(4.185, 2.659, new Rotation2d(150 * Math.PI / 180));
    public static final Pose2d reefE = new Pose2d(5.196, 2.890, new Rotation2d(210 * Math.PI / 180));
    public static final Pose2d reefF = new Pose2d(5.513, 3.053, new Rotation2d(210 * Math.PI / 180));
    public static final Pose2d reefG = new Pose2d(5.821, 4.112, new Rotation2d(270 * Math.PI / 180));
    public static final Pose2d reefH = new Pose2d(5.821, 4.400, new Rotation2d(270 * Math.PI / 180));
    public static final Pose2d reefI = new Pose2d(5.071, 5.218, new Rotation2d(330 * Math.PI / 180));
    public static final Pose2d reefJ = new Pose2d(4.782, 5.391, new Rotation2d(330 * Math.PI / 180));
    public static final Pose2d reefK = new Pose2d(3.791, 5.151, new Rotation2d(30 * Math.PI / 180));
    public static final Pose2d reefL = new Pose2d(3.473, 4.958, new Rotation2d(30 * Math.PI / 180));


    public static final PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
        
    public static PathPlannerPath toPath(ArrayList<Pose2d> poses) {
        return new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(poses),
                            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, poses.get(poses.size()-1).getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
    }
    public static PathPlannerPath toPath(Pose2d first, Pose2d second){
        ArrayList<Pose2d> x = new ArrayList<Pose2d>();
        x.add(first);
        x.add(second);

        return toPath(x);
    }

    public static SendableChooser<Pose2d> reef1() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.addOption("none", null);
        sc.addOption("A", reefA);
        sc.addOption("B", reefB);
        sc.addOption("C", reefC);
        sc.addOption("D", reefD);
        sc.addOption("E", reefE);
        sc.addOption("F", reefF);
        sc.addOption("G", reefG);
        sc.addOption("H", reefH);
        sc.addOption("I", reefI);
        sc.addOption("K", reefK);
        sc.addOption("L", reefL);
        sc.setDefaultOption("J", reefJ);

        return sc;
    }

    public static SendableChooser<Pose2d> reef2() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.addOption("none", null);
        sc.addOption("A", reefA);
        sc.addOption("B", reefB);
        sc.addOption("C", reefC);
        sc.addOption("D", reefD);
        sc.addOption("E", reefE);
        sc.addOption("F", reefF);
        sc.addOption("G", reefG);
        sc.addOption("H", reefH);
        sc.addOption("I", reefI);
        sc.addOption("J", reefJ);
        sc.addOption("L", reefL);
        sc.setDefaultOption("K", reefK);

        return sc;
    }

    public static SendableChooser<Pose2d> reef3() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.addOption("K", reefK);
        sc.addOption("A", reefA);
        sc.addOption("B", reefB);
        sc.addOption("C", reefC);
        sc.addOption("D", reefD);
        sc.addOption("E", reefE);
        sc.addOption("F", reefF);
        sc.addOption("G", reefG);
        sc.addOption("H", reefH);
        sc.addOption("I", reefI);
        sc.addOption("J", reefJ);
        sc.addOption("L", reefL);
        sc.setDefaultOption("none", null);

        return sc;
    }

    public static SendableChooser<Pose2d> humanPlayer1() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.setDefaultOption("top",humanPlayerT);
        sc.addOption("bottom",humanPlayerB);
        sc.addOption("none", null);

        return sc;
    }

    public static SendableChooser<Pose2d> humanPlayer2() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.setDefaultOption("top",humanPlayerT);
        sc.addOption("bottom",humanPlayerB);
        sc.addOption("none", null);

        return sc;
    }

    public static SendableChooser<Pose2d> humanPlayer3() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.addOption("top",humanPlayerT);
        sc.addOption("bottom",humanPlayerB);
        sc.setDefaultOption("none", null);    

        return sc;
    }

    public static SendableChooser<Pose2d> start() {
        init();
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();
        


        sc.addOption("top cage", startLeftCage);
        sc.addOption("middle cage", startMidCage);
        sc.addOption("bottom cage", startRightCage);
        if(!DriverStation.getLocation().isEmpty())
        switch (DriverStation.getLocation().getAsInt())
        {
            case 1:
            sc.setDefaultOption("left cage", startLeftCage);
            break;

            case 2:
            sc.setDefaultOption("middle cage", startMidCage);
            break;

            case 3:
            sc.setDefaultOption("right cage", startRightCage);
            break;
        }

        return sc;
    }

    public static Command startToReef() {
        if(reef1().getSelected() == null)
            return new Wait(15);
        Command c = AutoBuilder.pathfindToPose(reef1().getSelected(), constraints);
        c.addRequirements(drive);
        return c;
    }

    public static Command reefToHP1() {
        if(humanPlayer1().getSelected() == null)
            return new Wait(15);
        Command c = AutoBuilder.pathfindToPose(humanPlayer1().getSelected(), constraints);
        c.addRequirements(drive);
        return c;
    }

    public static Command hpToReef1() {
        if(reef2().getSelected() == null)
            return new Wait(15);
        Command c = AutoBuilder.pathfindToPose(reef2().getSelected(), constraints);
        c.addRequirements(drive);
        return c;
    }

    public static Command reefToHP2() {
        if(humanPlayer2().getSelected() == null)
            return new Wait(15);
        Command c = AutoBuilder.pathfindToPose(humanPlayer2().getSelected(), constraints);
        c.addRequirements(drive);
        return c;
    }

    public static Command hpToReef2() {
        if(reef3().getSelected() == null)
            return new Wait(15);
        Command c = AutoBuilder.pathfindToPose(reef3().getSelected(), constraints);
        c.addRequirements(drive);
        return c;
    }

    public static Command reefToHP3() {
        if(humanPlayer3().getSelected() == null)
            return new Wait(15);
        Command c = AutoBuilder.pathfindToPose(humanPlayer3().getSelected(), constraints);
        c.addRequirements(drive);
        return c;
    }

    public static SendableChooser<Constants.ELEVNUM> getLevel1() {
        SendableChooser<Constants.ELEVNUM> sc = new SendableChooser<Constants.ELEVNUM>();
        sc.addOption("L1", ELEVNUM.L1);
        sc.addOption("L2", ELEVNUM.L2);
        sc.addOption("L3", ELEVNUM.L3);
        sc.setDefaultOption("L4", ELEVNUM.L4);
        sc.addOption("none", null);

        return sc;
    }

    public static SendableChooser<Constants.ELEVNUM> getLevel2() {
        SendableChooser<Constants.ELEVNUM> sc = new SendableChooser<Constants.ELEVNUM>();
        sc.addOption("L1", ELEVNUM.L1);
        sc.addOption("L2", ELEVNUM.L2);
        sc.addOption("L3", ELEVNUM.L3);
        sc.setDefaultOption("L4", ELEVNUM.L4);
        sc.addOption("none", null);
        return sc;
    }

    public static SendableChooser<Constants.ELEVNUM> getLevel3(){
        SendableChooser<Constants.ELEVNUM> sc = new SendableChooser<Constants.ELEVNUM>();
        sc.addOption("L1", ELEVNUM.L1);
        sc.addOption("L2", ELEVNUM.L2);
        sc.addOption("L3", ELEVNUM.L3);
        sc.setDefaultOption("L4", ELEVNUM.L4);
        sc.addOption("none", null);

        return sc;
    }

    public static Command getAutoCommand(){
        Command result = Commands.sequence(
        startToReef(), 
        new SetElev(getLevel1().getSelected()),
        Commands.race(
            new RunOuttake(),
            new Wait(0.25)
        ),
        new SetElev(0),
        reefToHP1(),
        new SetElev(ELEVNUM.HP),
        Commands.race(
            new RunIntake(),
            new Wait(2)
        ),
        hpToReef1(),
         new SetElev(getLevel2().getSelected()),
        Commands.race(
            new RunOuttake(),
            new Wait(0.25)
        ),
        new SetElev(0),
        reefToHP2(),
        new SetElev(ELEVNUM.HP),
        Commands.race(
            new RunIntake(),
            new Wait(2)
        ),
        hpToReef2(),
        new SetElev(getLevel1().getSelected()),
       Commands.race(
           new RunOuttake(),
           new Wait(0.25)
       ),
        new SetElev(0),
        reefToHP3(),
        new SetElev(ELEVNUM.HP),
        Commands.race(
            new RunIntake(),
            new Wait(2)
        )
        );
        result.addRequirements(Intake.getInstance());
        result.addRequirements(Elevator.getInstance());
        result.addRequirements(Drivetrain.getInstance());
        return result;
    }

    

    public static void publishAll(){
        SmartDashboard.putData("1st coral pos", reef1());
        SmartDashboard.putData("2nd coral pos", reef2());
        SmartDashboard.putData("3rd coral pos", reef3());
        SmartDashboard.putData("1st HP station", humanPlayer1());
        SmartDashboard.putData("2nd HP station", humanPlayer2());
        SmartDashboard.putData("3rd HP station", humanPlayer3());
        SmartDashboard.putData("1st coral lvl", getLevel1());
        SmartDashboard.putData("2nd coral lvl", getLevel2());
        SmartDashboard.putData("3rd coral lvl", getLevel3());
        SmartDashboard.putData("Start pos", start());
    }

}