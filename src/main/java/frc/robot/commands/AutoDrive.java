/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.extraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WestCoastDrivetrain;

public class AutoDrive extends CommandBase {
  
  private final WestCoastDrivetrain drivetrain;

  //Define execution constraints (distance, direction, angle, timeout)
  private double targetDistance;
  private double targetAngle;
  private double direction;
  private double stopTime;

  //PID objects and variables
  private double angleCorrection, angleError, speedCorrection;
  private PIDControl pidAngle;
  private PIDControl pidSpeed;
  private PIDControl pidSpeedHigh; 

  //Timer stuff
  private double startTime;
  private Timer timer = new Timer();
  private double distanceTraveled;

  //Encoder 'saved' values
  private double leftEncoderStart;
  private double rightEncoderStart;

  //Constructor
  //@param dir: Direction is weird... -1 is forward, 1 is backward.  All other values are meaningless.
  public AutoDrive(WestCoastDrivetrain drive, double dis, double ang, double dir, double time) {

    //Set up drivetrain subsystem
    drivetrain = drive;
    addRequirements(drivetrain);

    //Initialize all constraint values
    targetDistance = dis;
    targetAngle = ang;
    direction = dir;
    stopTime = time;

    //Set up PID objects with values from Constants
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
    pidSpeed = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    //pidSpeedHigh = new PIDControl(kP_Speed_High, kI_Speed_High, kD_Speed_High);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //Start the timer
    distanceTraveled = 0.0;
    timer.start();
    startTime = timer.get();

    //Save current encoder values
    leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = drivetrain.getMasterRightEncoderPosition();

    //Zero the PID corrections
    angleCorrection = 0;
    angleError = 0;
    speedCorrection = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Run PID on angle and speed
    angleCorrection = pidAngle.run(drivetrain.getGyroAngle(), targetAngle);
    speedCorrection = pidSpeed.run(distanceTraveled, targetDistance);
    
    //Correct speed correction for over multiplying (speedCorrection is a factor of the ultimate speed, which cannot exceed 1.0)
    if (speedCorrection > 1.0) {
      speedCorrection = 1.0;
    } else if (speedCorrection < -1.0) {
      speedCorrection = -1.0;
    }

    //Actually drive
    drivetrain.autoDrive(speedCorrection * -direction * kAutoDriveSpeed - angleCorrection, speedCorrection * -direction * kAutoDriveSpeed + angleCorrection);
    
    //Track corrections from PID on SmartDash
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    SmartDashboard.putNumber("Speed Correction", speedCorrection);

    //Do math to figure out how far we have driven
    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));

    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    //Check both time and distance to figure out if we are done driving.
    boolean thereYet = false;
    double time = timer.get();

    //If we are within a given distance of the target distance, we call it good enough
    if (Math.abs(targetDistance - distanceTraveled) <= kAutoDistanceTolerance){

      thereYet = true;

      // else if(Math.abs(targetDistance - distanceTraveled) <= 24){

        //shifter.shiftDown();
      
      //}
    
    //Otherwise if we timeout we stop
    } else if (stopTime <= time - startTime){

      thereYet = true;
    }

    //Toss the distance up on SmartDash
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);

    return thereYet;

  }
}
