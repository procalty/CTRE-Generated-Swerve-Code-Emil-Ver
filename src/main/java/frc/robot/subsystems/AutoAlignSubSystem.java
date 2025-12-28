// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AutoAlignSubSystem extends SubsystemBase {

  private final NetworkTable limelightTable;

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
      this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

      xcontroller = new PIDController(2.0,0.0,0.1);
      ycontroller = new PIDController(2.0,0.0,0.1);
      rotcontroller = new PIDController(3.0,0.0,0.15);

      rotcontroller.enableContinuousInput(-180, 180);
        
      // Set tolerances
      xcontroller.setTolerance(POSITION_TOLERANCE);
      ycontroller.setTolerance(POSITION_TOLERANCE);
      rotcontroller.setTolerance(ROTATION_TOLERANCE);
    
  }

  // /**
  //  * @param targetID for the April Tag to align to
  //  * @param targetx for horizontal offeset
  //  * @param targety for vertical offset
  //  * @param targetArea for target area percentage covered
  //  */

  //  public Command alignToTarget(int targetID, double targetX, double targetY, double targetArea){
  //   return run(()->{
  //     double tv = LimelightTable.getEntry("tv").getDouble(0); // Valid target
  //     double tx = LimelightTable.getEntry("tx").getDouble(0); // Horizontal offset
  //     double ty = LimelightTable.getEntry("ty").getDouble(0); // Vertical offset
  //     double ta = LimelightTable.getEntry("ta").getDouble(0); // Area
  //     double tid = LimelightTable.getEntry("tid").getDouble(-1);  //tag ID

  //     if(tv == 1.0 && tid == targetID){
  //       double xerror = tx-targetX;
  //       double yerror = ta - targetArea;
  //       double roterror = tx - targetX;

  //       double straevel = xcontroller.calculate(tx,targetX);

  //       double forwardvel = ycontroller.calculate(ta,targetArea);

  //       double rotvel = -rotcontroller.calculate(tx,targetX);

  //       drivetrain.setControl(
  //         fieldCentricDrive
  //         .withVelocityX(forwardvel)
  //         .withVelocityY(straevel)
  //         .withRotationalRate(rotvel)
  //       );


  //     }
  //     else{
  //       drivetrain.setControl(
  //         fieldCentricDrive
  //         .withVelocityX(0)
  //         .withVelocityY(0)
  //         .withRotationalRate(0)
  //       );
  //     }

  //   }).until(() -> isAlignedBasic(targetID, targetX, targetArea));


  //  }


    /**
     * Command to align using MegaTag2 for better pose estimation
     * MegaTag2 uses multiple tags simultaneously for superior accuracy
     * @param targetID The AprilTag ID (optional, used for validation)
     * @param targetPose The desired pose on the field
     */

     public Command aligntotargetMT2(int targetID, Pose2d targetpose) {
      return run(() -> {
          double tv = limelightTable.getEntry("tv").getDouble(0);
  
          if (tv == 1.0 && isMT2PoseReliable()) {
              Pose2d currentPose = getRobotPose2d();
              
              double xVel = xcontroller.calculate(currentPose.getX(), targetpose.getX());
              double yVel = ycontroller.calculate(currentPose.getY(), targetpose.getY());
              double rotVel = rotcontroller.calculate(
                  currentPose.getRotation().getDegrees(), 
                  targetpose.getRotation().getDegrees()
              );
  
              drivetrain.setControl(fieldCentricDrive
                  .withVelocityX(xVel)
                  .withVelocityY(yVel)
                  .withRotationalRate(Math.toRadians(rotVel))
              );
          } else {
              drivetrain.setControl(fieldCentricDrive
                  .withVelocityX(0)
                  .withVelocityY(0)
                  .withRotationalRate(0)
              );
          }
      }).until(() -> isAlignedMegaTag(targetpose));
  }

      private boolean isAlignedMegaTag(Pose2d targetPose) {
        double tv = limelightTable.getEntry("tv").getDouble(0);
        
        if (tv != 1.0) {
            return false;
        }
        
        double[] botpose = getBotPoseBlue();
        
        if (botpose.length >= 6) {
            Pose2d currentPose = new Pose2d(
                botpose[0], 
                botpose[1], 
                Rotation2d.fromDegrees(botpose[5])
            );
            
            double xError = Math.abs(targetPose.getX() - currentPose.getX());
            double yError = Math.abs(targetPose.getY() - currentPose.getY());
            double rotError = Math.abs(targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());
            
            return xError < POSITION_TOLERANCE && 
                   yError < POSITION_TOLERANCE && 
                   rotError < ROTATION_TOLERANCE;
        }
        
        return false;
    }

    // ===== LIMELIGHT HELPER METHODS =====
    
    /**
     * Check if Limelight has a valid target
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }
    
    /**
     * Get horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
     */
    public double getTargetX() {
        return limelightTable.getEntry("tx").getDouble(0);
    }
    
    /**
     * Get vertical offset from crosshair to target (-24.85 to 24.85 degrees)
     */
    public double getTargetY() {
        return limelightTable.getEntry("ty").getDouble(0);
    }
    
    /**
     * Get target area (0% to 100% of image)
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }
    
    /**
     * Get AprilTag ID (or -1 if not an AprilTag)
     */
    public int getAprilTagID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }
    
    /**
     * Get robot pose from MegaTag2 (blue alliance origin)
     * MegaTag2 uses multi-tag observations for better accuracy
     * Returns [x, y, z, roll, pitch, yaw] in meters and degrees
     */
    public double[] getBotPoseBlue() {
        return limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
    }
    
    /**
     * Get robot pose from MegaTag2 (red alliance origin)
     */
    public double[] getBotPoseRed() {
        return limelightTable.getEntry("botpose_orb_wpired").getDoubleArray(new double[6]);
    }
    
    /**
     * Get robot pose as Pose2d using MegaTag2 (blue alliance)
     */
/**
 * Get robot pose for current alliance (automatically switches based on alliance)
 */
public Pose2d getRobotPose2d() {
    // Check which alliance we're on
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        // Use red alliance coordinates
        double[] pose = getBotPoseRed();
        if (pose.length >= 6) {
            return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
        }
    } else {
        // Use blue alliance coordinates (default)
        double[] pose = getBotPoseBlue();
        if (pose.length >= 6) {
            return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
        }
    }
    
    return new Pose2d(); // Return empty pose if no data
}
    
    /**
     * Get MegaTag2 data including timestamp and tag count
     * Returns [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
     */
    public double[] getBotPoseMT2Blue() {
        return limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
    }
    
    /**
     * Get number of AprilTags used in MegaTag2 solution
     */
    public int getMT2TagCount() {
        double[] pose = getBotPoseMT2Blue();
        return pose.length >= 8 ? (int) pose[7] : 0;
    }
    
    /**
     * Get tag span (area covered by tags in image) for MegaTag2
     * Higher span = more reliable pose
     */
    public double getMT2TagSpan() {
        double[] pose = getBotPoseMT2Blue();
        return pose.length >= 9 ? pose[8] : 0.0;
    }
    
    /**
     * Get average distance to tags in MegaTag2 solution
     */
    public double getMT2AvgTagDistance() {
        double[] pose = getBotPoseMT2Blue();
        return pose.length >= 10 ? pose[9] : 0.0;
    }
    
    /**
     * Check if MegaTag2 pose is reliable based on tag count and span
     */
    public boolean isMT2PoseReliable() {
        int tagCount = getMT2TagCount();
        double tagSpan = getMT2TagSpan();
        
        // Reliable if multiple tags with good coverage
        return tagCount >= 2 && tagSpan > 0.4;
    }
    
    /**
     * Get latency contribution of Limelight pipeline (ms)
     */
    public double getPipelineLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }
    
    /**
     * Get capture latency (ms)
     */
    public double getCaptureLatency() {
        return limelightTable.getEntry("cl").getDouble(0);
    }
    
    /**
     * Get total latency (pipeline + capture) in seconds
     */
    public double getTotalLatencySeconds() {
        return (getPipelineLatency() + getCaptureLatency()) / 1000.0;
    }
    
    /**
     * Set LED mode
     * @param mode 0=pipeline, 1=off, 2=blink, 3=on
     */
    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Set camera mode
     * @param mode 0=vision processor, 1=driver camera (increases exposure, disables vision processing)
     */
    public void setCameraMode(int mode) {
        limelightTable.getEntry("camMode").setNumber(mode);
    }
    
    /**
     * Set pipeline number
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Get current pipeline number
     */
    public int getPipeline() {
        return (int) limelightTable.getEntry("getpipe").getDouble(0);
    }
    
    /**
     * Take a snapshot (for documentation/debugging)
     * @param mode 0=stop snapshots, 1=take two snapshots per second
     */
    public void setSnapshot(int mode) {
        limelightTable.getEntry("snapshot").setNumber(mode);
    }
    
    /**
     * Calculate distance to target using target height and camera angle
     * @param targetHeightMeters Height of target above ground
     * @param cameraHeightMeters Height of camera above ground
     * @param cameraMountAngleDegrees Angle of camera from horizontal
     */
    public double getDistanceToTarget(double targetHeightMeters, double cameraHeightMeters, 
                                     double cameraMountAngleDegrees) {
        double ty = getTargetY();
        double angleToTargetDegrees = cameraMountAngleDegrees + ty;
        double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);
        
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToTargetRadians);
    }
    
    /**
     * Enable/disable driver mode (for easier driving, disables vision processing)
     */
    public void setDriverMode(boolean enabled) {
        setCameraMode(enabled ? 1 : 0);
        setLEDMode(enabled ? 1 : 3); // Turn off LEDs in driver mode
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
