// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.SpindexerSubsystem;

//import static edu.wpi.first.units.Units.Volt;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
public class FlipperCommand extends Command {


  private FeederSubsystem m_FeederSubsystem;
  //private SpindexerSubsystem m_SpindexerSubsystem;
  
  private int ticks;
  private boolean isRunning=false;
  //private double kRotationKd = 0;

  //private boolean m_tidFound = false;

  
  //private NetworkTable m_table;

  public FlipperCommand(FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_FeederSubsystem=feederSubsystem;
    //m_SpindexerSubsystem=spindexerSubsystem;
    addRequirements(m_FeederSubsystem);

    //NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //m_table = inst.getTable("Shoot");
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_table=NetworkTableInstance.getDefault().getTable("limelight");
    ticks =0;
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


    
    //m_turretSubsystem.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isRunning;
  }
}
