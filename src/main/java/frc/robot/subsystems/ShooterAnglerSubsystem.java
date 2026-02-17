package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ShooterAnglerSubsystem  extends SubsystemBase {

    private final SparkMax m_AnglerMotor = new SparkMax(SubsystemConstants.kAnglerId, MotorType.kBrushless);
    public SparkRelativeEncoder m_AnglerEncoder = (SparkRelativeEncoder) m_AnglerMotor.getEncoder();
    public ShooterAnglerSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_AnglerMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle Shooter Position", (m_AnglerEncoder.getPosition()/16.0));
    }
    public void setSpeed(Supplier<Double> speed) {
        m_AnglerMotor.set(speed.get());
        

        
    }
}
