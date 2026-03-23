// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FlipperCommand extends Command {

  private FeederSubsystem m_FeederSubsystem;  
  private int ticks;
  private boolean isRunning=false;

  public FlipperCommand(FeederSubsystem feederSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_FeederSubsystem=feederSubsystem;
    addRequirements(m_FeederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ticks = 0;
    //tweak on a per id basis, just a generic value for april tag
    SmartDashboard.putBoolean("flipper is done", false);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(ticks<49){
      m_FeederSubsystem.setFlipperSpeed(-0.6);
      ticks++;
    }
    else{
      m_FeederSubsystem.setFlipperSpeed(0.0);
      isRunning=true;
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_FeederSubsystem.setFlipperSpeed(0.0);
      SmartDashboard.putBoolean("flipper is done", true);
      isRunning=false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isRunning;
  }
}