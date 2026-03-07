package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;
public class RotTurretCommand extends Command {
   private TurretSubsystem m_turret;
   private double m_speed = 0.0;
   private Supplier<Double> m_rightX;
   
   public RotTurretCommand(TurretSubsystem Turret, Supplier<Double> rightX){
    addRequirements(Turret);
    m_turret = Turret;
    m_rightX=rightX;
    
   }
    //only runs once
    @Override
    public void initialize() {
        m_turret.RotateTurret(m_rightX);
    }
//everyt time it is ran
    @Override
  public void execute() {
    //m_turret.RotateTurret(m_rightX);
    m_speed=m_turret.m_TurretEncoder.getPosition()/16.0;
    double target = LimelightHelpers.getTX("limelight")/20.0;
    double scaleSpeed=2.0;
    m_speed=Math.abs(target-m_speed)/scaleSpeed;
    if (m_speed>0.3){
      m_speed=0.3;
    }
    if(target<(m_turret.m_TurretEncoder.getPosition()/16.0)){
      m_speed*=-1.0;
    }
    m_turret.RotateTurret(()->m_speed);
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
