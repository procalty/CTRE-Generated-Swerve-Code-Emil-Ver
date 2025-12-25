// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.PDPSim;


public class AutoAlignSubSystem extends SubsystemBase {

  private final NetworkTable LimelightTable;

  private final PIDController xcontroller;
  private final PIDController ycontroller;
  private final PIDController rotcontroller;

  private final CommandSwerveDrivetrain drivetrain;

   private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();

   private static final double POSITION_TOLERANCE = 0.05; // meters
   private static final double ROTATION_TOLERANCE = 2.0; // degrees


  /** Creates a new AutoAlignSubSystem. */
  public AutoAlignSubSystem(CommandSwerveDrivetrain drivetrain) {
      this.drivetrain = drivetrain;
      this.LimelightTable = NetworkTableInstance.getDefault().getTable("limelight");

      xcontroller = new PIDController(2.0,0.0,0.1);
      ycontroller = new PIDController(2.0,0.0,0.1);
      rotcontroller = new PIDController(3.0,0.0,0.15);

      rotcontroller.enableContinuousInput(-180, 180);
        
      // Set tolerances
      xcontroller.setTolerance(POSITION_TOLERANCE);
      ycontroller.setTolerance(POSITION_TOLERANCE);
      rotcontroller.setTolerance(ROTATION_TOLERANCE);
    
  }

  /**
   * @param targetID for the April Tag to align to
   * @param targetx for horizontal offeset
   * @param targety for vertical offset
   * @param targetArea for target area percentage covered
   */

   public Command alignToTarget(int targetID, double targetX, double targetY, double targetArea){
    return run(()->{
      double tv = LimelightTable.getEntry("tv").getDouble(0); // Valid target
      double tx = LimelightTable.getEntry("tx").getDouble(0); // Horizontal offset
      double ty = LimelightTable.getEntry("ty").getDouble(0); // Vertical offset
      double ta = LimelightTable.getEntry("ta").getDouble(0); // Area
      double tid = LimelightTable.getEntry("tid").getDouble(-1);  //tag ID

      if(tv == 1.0 && tid == targetID){
        double xerror = tx-targetX;
        double yerror = ta - targetArea;
        double roterror = tx - targetX;

        double straevel = xcontroller.calculate(tx,targetX);

        double forwardvel = ycontroller.calculate(ta,targetArea);

        double rotvel = -rotcontroller.calculate(tx,targetX);

        drivetrain.setControl(
          fieldCentricDrive
          .withVelocityX(forwardvel)
          .withVelocityY(straevel)
          .withRotationalRate(rotvel)
        );


      }
      else{
        drivetrain.setControl(
          fieldCentricDrive
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0)
        );
      }

    }).until(() -> isAlignedBasic(targetID, targetX, targetArea));


   }

  private boolean isAlignedBasic(int targetID, double targetX, double targetArea) {
    double tv = LimelightTable.getEntry("tv").getDouble(0);
    double tid = LimelightTable.getEntry("tid").getDouble(-1);
    
    if (tv != 1.0 || tid != targetID) {
        return false;
    }
    
    double tx = LimelightTable.getEntry("tx").getDouble(0);
    double ta = LimelightTable.getEntry("ta").getDouble(0);
    
    return Math.abs(tx - targetX) < ROTATION_TOLERANCE && 
           Math.abs(ta - targetArea) < 1.0; // 1% area tolerance
}

public boolean hasTarget() {
  return LimelightTable.getEntry("tv").getDouble(0) == 1.0;
}

/**
* Get horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
*/
public double getTargetX() {
  return LimelightTable.getEntry("tx").getDouble(0);
}

/**
* Get vertical offset from crosshair to target (-24.85 to 24.85 degrees)
*/
public double getTargetY() {
  return LimelightTable.getEntry("ty").getDouble(0);
}

/**
* Get target area (0% to 100% of image)
*/
public double getTargetArea() {
  return LimelightTable.getEntry("ta").getDouble(0);
}

/**
* Get AprilTag ID (or -1 if not an AprilTag)
*/
public int getAprilTagID() {
  return (int) LimelightTable.getEntry("tid").getDouble(-1);
}

public void setLEDMode(int mode) {
  LimelightTable.getEntry("ledMode").setNumber(mode);
}

public void setPipeline(int pipeline) {
  LimelightTable.getEntry("pipeline").setNumber(pipeline);
}







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
