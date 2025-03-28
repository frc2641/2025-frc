package frc.team2641.robot2025.helpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Limelight;
import frc.team2641.robot2025.commands.auto.AutoAngle;

/** season-specific interface between robotcontainer and AutoAngle.  */
public class LimelightHelper {
    
    private double targetAngle;
    private double oppositeAngle;

    public static enum LIMELIGHT_ELEMENT {
        REEF,
        CORAL_STATION
    };


    /** @param degrees */
    public LimelightHelper(int target, int op){
        this.targetAngle = target;
        this.oppositeAngle = op;
    }

    /** @param radians */
    /** @param rad is solely to describe your mental state */
    public LimelightHelper(double target, double opposite, boolean rad){
        this.targetAngle = target;
        this.oppositeAngle = opposite;
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    public double getOppositeAngle(){
        return oppositeAngle;
    }

    public void flip(){
        targetAngle *= -1;
        oppositeAngle *= -1;
    }

    public static LimelightHelper getLimeLightHelper(LIMELIGHT_ELEMENT x){
        if(x == null)
            return null;
        switch (x) {
            case REEF:
                return getReef(); 
            case CORAL_STATION:
                    
                return getCoralStation(); 

            default:
                return null;
        }
    }

    public static Command angle(){return new AutoAngle(null, false);}
// NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    private static LimelightHelper getReef(){
        switch ((int)Limelight.getFiducialID("")) {
            case 0:
                
                break;
        
            default:
                break;
        }
        return new LimelightHelper(0, 0);
    }

    private static LimelightHelper getCoralStation(){
        return new LimelightHelper(0,0);
    }


}
