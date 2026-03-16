package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;
public class RotTurretCommand extends Command {
   private TurretSubsystem m_turret;
   private double m_speed = 0.0;
   //private Supplier<Double> m_rightX;
   private double m_tid;
   
   public RotTurretCommand(TurretSubsystem Turret){
    addRequirements(Turret);
    m_turret = Turret;
    
    
   }
    //only runs once
    @Override
    public void initialize() {
        //m_turret.RotateTurret(m_rightX);
         double tid = LimelightHelpers.getFiducialID("limelight");
        m_tid=tid;
    }
//everyt time it is ran
    @Override
  public void execute() {
    m_tid = LimelightHelpers.getFiducialID("limelight");
    //m_turret.RotateTurret(m_rightX);
    double gearRatio=16.0;
      if ((m_tid!=-1 && m_tid!=0)) {
        m_speed=m_turret.m_TurretEncoder.getPosition()/gearRatio;
        //m_speed= m_turret.getAbsPos();
        double scaleToOne=20.0;
        double target = LimelightHelpers.getTX("limelight")/scaleToOne;// *180.0 //for degs
        //double noEnterWest=315.0;
        //double noEnterEast=45.0;
        //target+=180.0;
        if((Math.abs(target-m_speed))>.05){
          double scaleSpeed=2.0;
          m_speed=Math.abs(target-m_speed)/scaleSpeed;

          if (m_speed>0.3){
            m_speed=0.3;
          }

          if(target<(m_turret.m_TurretEncoder.getPosition()/gearRatio)){
            m_speed*=-1.0;
          }
          //if(((m_turret.getAbsPos()>0)&& (m_speed<0.0)))
          /*if(m_speed<0.0){
            if(){

            }
          }*/
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
