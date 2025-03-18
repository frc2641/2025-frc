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
import frc.team2641.robot2025.Constants.ArmPositions;
import frc.team2641.robot2025.commands.Wait;
import frc.team2641.robot2025.commands.superStructure.SetArmTarget;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.commands.RunIntake;
import frc.team2641.robot2025.commands.shifts.ReverseIntake;

public class Autos {
    
    public static final Pose2d startLeftCage = new Pose2d(7.592,7.239, new Rotation2d(180 * Math.PI / 180));
    public static final Pose2d startMidCage = new Pose2d(7.592, 6.180, new Rotation2d(180 * Math.PI / 180));
    public static final Pose2d startRightCage = new Pose2d(7.592, 5.083, new Rotation2d(180 * Math.PI / 180));

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

    private static final ArrayList<SendableChooser<Pose2d>> reefs = getReefs();

    private static ArrayList<SendableChooser<Pose2d>> getReefs(){
        ArrayList<SendableChooser<Pose2d>> x = new ArrayList<SendableChooser<Pose2d>>();
        x.add(reef1());
        x.add(reef2());
        x.add(reef3());
        return x;
    }

    private static final ArrayList<SendableChooser<Pose2d>> HPs = getHPs();

    private static ArrayList<SendableChooser<Pose2d>> getHPs(){
        ArrayList<SendableChooser<Pose2d>> x = new ArrayList<SendableChooser<Pose2d>>();
        x.add(humanPlayer1());
        x.add(humanPlayer2());
        x.add(humanPlayer3());
        return x;
    }

    private static final ArrayList<SendableChooser<ArmPosition>> levels = getLevels();

    private static ArrayList<SendableChooser<ArmPosition>> getLevels(){
        ArrayList<SendableChooser<ArmPosition>> x = new ArrayList<SendableChooser<ArmPosition>>();
        x.add(getLevel1());
        x.add(getLevel2());
        x.add(getLevel3());
        return x;
    }

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
        sc.setDefaultOption("J", reefJ);
        sc.addOption("K", reefK);
        sc.addOption("L", reefL);

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
        sc.setDefaultOption("K", reefK);
        sc.addOption("L", reefL);

        return sc;
    }

    public static SendableChooser<Pose2d> reef3() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.setDefaultOption("none", null);
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
        sc.addOption("K", reefK);
        sc.addOption("L", reefL);

        return sc;
    }

    public static SendableChooser<Pose2d> humanPlayer1() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.addOption("none", null);
        sc.setDefaultOption("top",humanPlayerT);
        sc.addOption("bottom",humanPlayerB);

        return sc;
    }

    public static SendableChooser<Pose2d> humanPlayer2() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.addOption("none", null);
        sc.setDefaultOption("top",humanPlayerT);
        sc.addOption("bottom",humanPlayerB);

        return sc;
    }

    public static SendableChooser<Pose2d> humanPlayer3() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();

        sc.setDefaultOption("none", null);  
        sc.addOption("top",humanPlayerT);
        sc.addOption("bottom",humanPlayerB);  

        return sc;
    }

    public static SendableChooser<Pose2d> start() {
        SendableChooser<Pose2d> sc = new SendableChooser<Pose2d>();
        
        sc.addOption("left cage", startLeftCage);
        sc.addOption("middle cage", startMidCage);
        sc.addOption("right cage", startRightCage);
        switch (DriverStation.getLocation().getAsInt()){
            case 1:
            sc.setDefaultOption("left cage", startLeftCage);
            case 2:
            sc.setDefaultOption("middle cage", startMidCage);
            case 3:
            sc.setDefaultOption("right cage", startRightCage);
        }

        return sc;
    }

    public static Command startToReef() {
        return AutoBuilder.pathfindToPose(reef1().getSelected(), constraints);
    }

    public static Command reefToHP1() {
        return AutoBuilder.pathfindToPose(humanPlayer1().getSelected(), constraints);
    }

    public static Command hpToReef1() {
        return AutoBuilder.pathfindToPose(reef2().getSelected(), constraints);
    }

    public static Command reefToHP2() {
        return AutoBuilder.pathfindToPose(humanPlayer2().getSelected(), constraints);
    }

    public static Command hpToReef2() {
        return AutoBuilder.pathfindToPose(reef3().getSelected(), constraints);
    }

    public static Command reefToHP3() {
        return AutoBuilder.pathfindToPose(humanPlayer3().getSelected(), constraints);
    }

    public static SendableChooser<ArmPosition> getLevel1() {
        SendableChooser<ArmPosition> sc = new SendableChooser<ArmPosition>();
        sc.addOption("none", null);
        sc.addOption("L1", ArmPositions.L1);
        sc.addOption("L2", ArmPositions.L2);
        sc.addOption("L3", ArmPositions.L3);
        sc.setDefaultOption("L4", ArmPositions.L4);

        return sc;
        
    }

    public static SendableChooser<ArmPosition> getLevel2() {
        SendableChooser<ArmPosition> sc = new SendableChooser<ArmPosition>();
        sc.addOption("none", null);
        sc.addOption("L1", ArmPositions.L1);
        sc.addOption("L2", ArmPositions.L2);
        sc.addOption("L3", ArmPositions.L3);
        sc.setDefaultOption("L4", ArmPositions.L4);
        return sc;
    }

    public static SendableChooser<ArmPosition> getLevel3(){
        SendableChooser<ArmPosition> sc = new SendableChooser<ArmPosition>();
        sc.addOption("none", null);
        sc.addOption("L1", ArmPositions.L1);
        sc.addOption("L2", ArmPositions.L2);
        sc.addOption("L3", ArmPositions.L3);
        sc.setDefaultOption("L4", ArmPositions.L4);

        return sc;
    }

    public static Command[] getCommands(){
        Command[] x = new Command[12];
        x[0] = startToReef();
        x[1] = elevAndShoot(getLevel1().getSelected());
        x[2] = reefToHP1();
        x[3] = specialIntake();
        x[4] = hpToReef1();
        x[5] = elevAndShoot(getLevel2().getSelected());
        x[6] = reefToHP2();
        x[7] = specialIntake();
        x[8] = hpToReef2();
        x[9] = elevAndShoot(getLevel3().getSelected());
        x[10] = reefToHP3();
        x[11] = specialIntake();

        return x;
    }

    public static Command elevAndShoot(ArmPosition x){
        Command results = Commands.none();
        results.andThen(new SetArmTarget(x));
        results.andThen(new RunIntake());
        results.alongWith(new ReverseIntake());
        return results;
    }

    public static Command specialIntake(){
        Command result = Commands.none();

        result.alongWith(new RunIntake());
        result.alongWith(new Wait(2));

        return result;
    }    

    public static Command getAutoCommand() {
        Command result = Commands.none();
        Command[] toAdd = getCommands();
        for(Command x : toAdd){
            if(x == null)
                return result;
            result.andThen(x);   
        }
        return result;
}

    public static void publishAll() {
        SmartDashboard.putData("Start pos", start());
        SmartDashboard.putData("1st reef coral location", reef1());
        SmartDashboard.putData("1st reef coral level", getLevel1());
        SmartDashboard.putData("1st human player station", humanPlayer1());
        SmartDashboard.putData("2nd reef coral location", reef2());
        SmartDashboard.putData("2nd reef coral level", getLevel2());
        SmartDashboard.putData("2nd human player station", humanPlayer2());
        SmartDashboard.putData("3rd reef coral location", reef3());
        SmartDashboard.putData("3rd reef coral level", getLevel3());
        SmartDashboard.putData("3rd human player station", humanPlayer3());
    }

}