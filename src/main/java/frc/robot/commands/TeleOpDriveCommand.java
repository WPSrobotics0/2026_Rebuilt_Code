// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends Command {
    private DriveSubsystem m_driveSubsystem;
    private Supplier<Double> m_xJoystickSupplier;
    private Supplier<Double> m_yJoystickSupplier;
    private Supplier<Double> m_turnJoystickSupplier;
    private Supplier<Boolean> m_isTeleopEnabled;
    private Supplier<Boolean> m_isFieldRelative;
    public Supplier<Boolean> m_isBump;
    private int ticks;
    private int inversion;
    private boolean isInversion;


    public TeleOpDriveCommand(DriveSubsystem driveSubsystem,
            Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> turnJoystick,
            Supplier<Boolean> isTeleopEnabled, Supplier<Boolean> isFieldRelative) {
        m_driveSubsystem = driveSubsystem;

        m_xJoystickSupplier = xJoystick;
        m_yJoystickSupplier = yJoystick;
        m_turnJoystickSupplier = turnJoystick;
        m_isTeleopEnabled = isTeleopEnabled;
        m_isFieldRelative=isFieldRelative;
        ticks=0;
        m_isBump=()->false;
        inversion = 1;
        isInversion = false;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double angle = m_driveSubsystem.getAngle().getDegrees()%360;
        if ((Math.abs(angle/45.0f)%2<0.50)&&isInversion == false){
            isInversion = true;
            inversion = -1;
        }
        SmartDashboard.putNumber("45 Angle", Math.abs(angle/45.0f)%2);
        if (m_isTeleopEnabled.get()){
            if(m_isBump.get()==false){
            m_driveSubsystem.drive(m_xJoystickSupplier.get(), m_yJoystickSupplier.get(), m_turnJoystickSupplier.get(),
                        m_isFieldRelative.get());}
            else{
                if((Math.abs(angle/45.0f)%2>0.90) || Math.abs(angle/45.0f)%2<0.80){
                     m_driveSubsystem.drive(m_xJoystickSupplier.get(), m_yJoystickSupplier.get(), 2*inversion,
                    m_isFieldRelative.get());
                    //ticks++;
                }
                    else {
                        //ticks=0;
                        m_isBump=()->false;
                        isInversion = false;
                        inversion = 1;
                    }
                        }
                    }
        
    }
    public void flipBump(){
        m_isBump=()->true;
    }
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
