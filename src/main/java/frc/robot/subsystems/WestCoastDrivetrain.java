/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WestCoastDrivetrain extends GenericDrivetrain {
  /**
   * Creates a new WestCoastDrivetrain.
   */

  //Gyro
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  //Encoders
  private CANEncoder l1_encoder;
  private CANEncoder l2_encoder;
  private CANEncoder R1_encoder;
  private CANEncoder R2_encoder;


  //Motor Controllers
  // private WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(LEFT_MOTOR_1);
  // private WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(LEFT_MOTOR_2);
  private CANSparkMax leftMotor1 = new CANSparkMax(LEFT_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(LEFT_MOTOR_2, MotorType.kBrushless);
  private WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(LEFT_MOTOR_3);

  // private WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RIGHT_MOTOR_1);
  // private WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RIGHT_MOTOR_2);
  private CANSparkMax rightMotor1 = new CANSparkMax(RIGHT_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(RIGHT_MOTOR_2, MotorType.kBrushless);
  private WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RIGHT_MOTOR_3);
  
  private SpeedControllerGroup leftMotorGroup;
  private SpeedControllerGroup rightMotorGroup;

  private DifferentialDrive drivetrain;

  public WestCoastDrivetrain() {

    //Create motor groups and drivetrain
    if (MOTOR_COUNT == 2)
    {
      leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2);
      rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2);
    }
    else if (MOTOR_COUNT == 3)
    {
      leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
      rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
    }

    drivetrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    drivetrain.setRightSideInverted(true);

    initEncoders();
    gyro.calibrate();

    SmartDashboard.putNumber("Zero Gyro", 0);
    SmartDashboard.putNumber("Reset Encoders", 0);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", l1_encoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", R1_encoder.getPosition());

    SmartDashboard.putNumber("Left Encoder Velocity", l1_encoder.getVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", R1_encoder.getVelocity());

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());

    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if(zeroGyro == 1){
      zeroGyro();
      SmartDashboard.putNumber("Zero Gyro", 0);
    }

    double resetEncoders = SmartDashboard.getNumber("Reset Encoders", 0);
    if(resetEncoders == 1){
      resetEncoders();
      SmartDashboard.putNumber("Reset Encoders", 0);
    }

  }

  private void initEncoders(){

    //Initialize and set encoders to zero position
    l1_encoder = leftMotor1.getEncoder();
    l2_encoder = leftMotor2.getEncoder();
    R1_encoder = rightMotor1.getEncoder();
    R2_encoder = rightMotor2.getEncoder();

    l1_encoder.setPosition(0);
    l2_encoder.setPosition(0);
    R1_encoder.setPosition(0);
    R2_encoder.setPosition(0);

  }

  public double getMasterLeftEncoderPosition(){
    return l1_encoder.getPosition();
  }

  public double getMasterRightEncoderPosition(){
    return R1_encoder.getPosition();
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public void zeroGyro(){
    gyro.reset();
  }

  public void resetEncoders(){
    l1_encoder.setPosition(0);
    l2_encoder.setPosition(0);
    R1_encoder.setPosition(0);
    R2_encoder.setPosition(0);
  }

  @Override
  public void drive(double leftJoyX, double leftJoyY, double rightJoyX, double rightJoyY){

    double speedcap = .8;

    if(DIRECTION_MULTIPLIER == 1){

      drivetrain.tankDrive(speedcap * DIRECTION_MULTIPLIER * leftJoyY, speedcap * DIRECTION_MULTIPLIER * rightJoyY);   
    
    }
    else{

      drivetrain.tankDrive(speedcap * DIRECTION_MULTIPLIER * rightJoyY, speedcap * DIRECTION_MULTIPLIER * leftJoyY); 
    
    }
  }

  @Override
  public void autoDrive(double leftSpeed, double rightSpeed){

    //Pretty self-explanatory, just feed the drivetrain whatever speed we give it
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
  
  //Direction switching for manueverabiltiy
  public void switchDirection(){
    DIRECTION_MULTIPLIER *= -1;
    
  }

  //Stop the robot
  public void stopDrive(){
    drivetrain.tankDrive(0, 0);
  }

}
