// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.controller.PIDController;
public class ShootCommand extends Command {


  private TurretSubsystem m_turretSubsystem;
  private double m_targetZ; 
  private double m_tid;
  private double m_Forward;
  private double kSpeedKp = 0.125;
  private double kSpeedKi = 0;
  private double kSpeedKd = 0;

  private double m_driveForwardTarget;
  PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

  private boolean m_tidFound = false;

  
  private NetworkTable m_table;

  public ShootCommand(TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turretSubsystem = turretSubsystem;

    addRequirements(m_turretSubsystem);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_table = inst.getTable("Shoot");
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_table=NetworkTableInstance.getDefault().getTable("limelight");
    double tid = LimelightHelpers.getFiducialID("limelight");
    m_tid=tid;
    
    //tweak on a per id basis, just a generic value for april tag
    m_targetZ =0;

    

    m_xSpeedController.setSetpoint(m_targetZ);

  }

  
  private double forward(){
    double kP=.1;
    m_Forward=LimelightHelpers.getTY("limelight") *kP;

    return m_xSpeedController.calculate(m_Forward)*(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_tid = LimelightHelpers.getFiducialID("limelight");
  
    SmartDashboard.putNumber(" command tid", m_tid);
   
    if (m_tid!=-1 && m_tid!=0) {
      //FORWARD SPEED
      m_driveForwardTarget=forward(); 

      SmartDashboard.putBoolean("isValidId", true);
      SmartDashboard.putNumber("drivespeed", m_driveForwardTarget);
      m_turretSubsystem.setIntakeSpeed(()->1.0);
      
    
    } else {
      SmartDashboard.putBoolean("isValidId", false);
      m_tidFound=false;
      //m_turretSubsystem.drive(0, 0, 0, true);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
      m_turretSubsystem.setIntakeSpeed(()->0.0);
    //m_turretSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tidFound == false;
  }
}
