// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ArmDistanceCalculationTests;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.ArmPoseCalculator;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** Add your docs here. */
public class ArmDistanceCalculationTests {
    @Test
    public void test2dDistance_straightX1Y0Z0_distance1(){
        ArmPoseCalculator apc = new ArmPoseCalculator();
        apc.setRobotPose(new Translation3d(0.0,0.0,0.0));
        apc.setTargetPose(new Translation3d(1.0,0.0,0.0));

        double actual = apc.get2dDistance();
        
        assertEquals(1.0,actual,"expected 2d distance to be 1.0, got " + actual);
    }

    @Test
    public void test2dDistance_straightX1Y1Z0_distanceRoot2(){
        ArmPoseCalculator apc = new ArmPoseCalculator();
        apc.setRobotPose(new Translation3d(0.0,0.0,0.0));
        apc.setTargetPose(new Translation3d(1.0,1.0,0.0));
        
        double expected = Math.sqrt(3.0);
        double actual = apc.get2dDistance();
        
        assertEquals(expected,actual,"expected 2d distance to be " + expected + ", got " + actual);
    }
    @Test
    public void testTurretAngle_straightX0Y0Z0(){
        ArmPoseCalculator apc = new ArmPoseCalculator();
        apc.setRobotPose(new Translation3d(0.0, 0.0, 0.3));
        apc.setTargetPose(new Translation3d(1.0, 0.0, 0.3));
        double expected = Math.toRadians(360.0); 
        double actual = Math.toRadians(apc.getElevationAngleDegrees());
        assertEquals(expected, actual, "expected arm angle to be " + expected + "- got " + actual);
    }
    @Test
    public void testTurretAngle_straightX0Y0Z1(){
        ArmPoseCalculator apc = new ArmPoseCalculator();
        apc.setRobotPose(new Translation3d(0.0, 0.0, 0.3));
        apc.setTargetPose(new Translation3d(0.0, 0.0, 1.3));
        double expected = Math.toRadians(90.0);
        double actual = Math.toRadians(apc.getElevationAngleDegrees());
        assertEquals(expected, actual, "expected arm angle to be " + expected + "- got " + actual);
    }
    @Test
    public void testAdd2(){
        assertEquals(1,1,"aeiou");
    }
}
