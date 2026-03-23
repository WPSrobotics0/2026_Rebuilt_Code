package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;
public class RotTurretCommand extends Command {
   private TurretSubsystem m_turret;
   private double m_speed = 0.0;
   private double m_tid;

   public RotTurretCommand(TurretSubsystem Turret){
    addRequirements(Turret);
    m_turret = Turret;
   }
    
  @Override
  public void initialize() {
    double tid = LimelightHelpers.getFiducialID("limelight");
    m_tid=tid;
  }

  @Override
  public void execute() {
    m_tid = LimelightHelpers.getFiducialID("limelight");
    double gearRatio=16.0;
      if ((m_tid!=-1 && m_tid!=0)) {
        m_speed=m_turret.m_TurretEncoder.getPosition()/gearRatio;
        double scaleToOne=20.0;
        double target = LimelightHelpers.getTX("limelight")/scaleToOne;
        
        if((Math.abs(target-m_speed))>.05){
          double scaleSpeed=1.5;
          m_speed=Math.abs(target-m_speed)/scaleSpeed;

          if (m_speed>0.6){
            m_speed=0.6;
          }

          if(target<(m_turret.m_TurretEncoder.getPosition()/gearRatio)){
            m_speed*=-1.0;
          }
          
          m_turret.RotateTurret(()->m_speed);
        }

        else{
          m_turret.RotateTurret(()->0.0);
        }

        SmartDashboard.putNumber("rot turret target pos", target);
        SmartDashboard.putNumber("rot turret pos", (m_turret.m_TurretEncoder.getPosition()/gearRatio));
        SmartDashboard.putNumber("rot turret speed", m_speed);
    }
    else{
      m_turret.RotateTurret(()->0.0);
    }
    
  }
  @Override
  public void end(boolean interrupted) {
    m_turret.RotateTurret(()->0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}