package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterAnglerSubsystem;
public class AngleShooterCommand extends Command {
   private ShooterAnglerSubsystem m_Angler;
   private double m_speed = 0.0;
   public AngleShooterCommand(ShooterAnglerSubsystem Angler){
    addRequirements(Angler);
    m_Angler = Angler;
    
    
   }
    //only runs once
    @Override
    public void initialize() {
        //m_Angler.Rotateshooter(()->1.0);
    }
//everyt time it is ran
    @Override
  public void execute() {
    //m_Angler.Rotateshooter(()->1.0);
    m_speed=m_Angler.m_AnglerEncoder.getPosition()/16.0;
    double target = LimelightHelpers.getTY("limelight")/20.0;
    double scaleSpeed=2.0;
    m_speed=Math.abs(target-m_speed)/scaleSpeed;
    if (m_speed>1.0){
      m_speed=1.0;
    }
    SmartDashboard.putNumber("angle rotate position", m_Angler.m_AnglerEncoder.getPosition()/16.0);
    SmartDashboard.putNumber("angle rotate speed", m_speed);
    if (target-0.05>m_Angler.m_AnglerEncoder.getPosition()/16.0) {
        
         m_Angler.setSpeed(m_speed);

        }
        else if (target+0.05<m_Angler.m_AnglerEncoder.getPosition()/16.0) {
          m_speed=-1*m_speed;
            m_Angler.setSpeed(m_speed);
        }
        else{
            m_Angler.setSpeed(0.0);
        } 
  }
   @Override
  public void end(boolean interrupted) {
     m_Angler.setSpeed(0.0);
}
   @Override
  public boolean isFinished() {
    return false;
  }
}
