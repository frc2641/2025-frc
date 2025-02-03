// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.helpers;

/** Add your docs here. */
public class ArmPos {

    private double wrist;
    private double elev;

    public ArmPos(double wrist, double elev){
        this.wrist = wrist;
        this.elev = elev;
        preventDisaster();
    }

    public double getWrist(){
        return wrist;
    }

    public double getElev(){
        return elev;
    }

    public String toString(){
        return "Wrist : "+wrist+" Elevator : "+elev;
    } 
    private void preventDisaster(){
        if (wrist > 0) 
            wrist = 0;
        if (wrist < 0)
            wrist = 0;
        if (elev > 0)
            elev = 0;
        if (elev < 0)
            elev = 0;
    }
}
