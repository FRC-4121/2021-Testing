/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericDrivetrain;

public class DriveWithXboxCommand extends CommandBase {
  /**
   * Creates a new DriveWithJoysticksCommand.
   */
  private GenericDrivetrain drive;
  private XboxController xbox = new XboxController(XBOX_PORT);

  public DriveWithXboxCommand(GenericDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    drive = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(DrivetrainConstants.DRIVETRAIN_TYPE == 0){
      
      //West Coast
    drive.drive(xbox.getX(Hand.kLeft), -xbox.getY(Hand.kLeft), xbox.getX(Hand.kRight), -xbox.getY(Hand.kRight));
    // }
    // else if(DrivetrainConstants.DRIVETRAIN_TYPE == 1){
      
    //   //Mecanum
    //   drive.drive(xbox.getX(Hand.kLeft), xbox.getY(Hand.kLeft), xbox.getX(Hand.kRight), false);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
