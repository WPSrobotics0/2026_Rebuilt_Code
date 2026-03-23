package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import java.util.function.Supplier;
import com.andymark.jni.AM_CAN_HexBoreEncoder;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class TurretSubsystem  extends SubsystemBase {

    private final SparkMax m_TurretMotor = new SparkMax(SubsystemConstants.kTurretId, MotorType.kBrushless);
    public AM_CAN_HexBoreEncoder absEncoder = new AM_CAN_HexBoreEncoder(40);
    public SparkRelativeEncoder m_TurretEncoder = (SparkRelativeEncoder) m_TurretMotor.getEncoder();
    public double targetPoint=0.0;
    
    public TurretSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_TurretMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        absEncoder.resetReportPeriod();
        absEncoder.setZeroHere();  
    }

    @Override
    public void periodic() {
        double degrees = absEncoder.getAngleDegrees();
        double degreesPerSec =  absEncoder.getVelocityDegPerSec();
        SmartDashboard.putNumber("hex bore angleDegrees", degrees);
        SmartDashboard.putNumber("hexBoreVelocity", degreesPerSec);
    }

    public double getAbsPos(){
        return absEncoder.getAngleDegrees();

    }

    public void RotateTurret(Supplier<Double> speed) {
        m_TurretMotor.set(speed.get()/5.0);
    }
}