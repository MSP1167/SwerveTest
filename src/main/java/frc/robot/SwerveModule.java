package frc.robot;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Util;

public class SwerveModule {

    public TalonFX Drive;
    public TalonFX Turn;
    //Constants.FALCON500_ENCODER_TICKS
    private double TICKS_PER_DEGREE =  44032 / 360;
    private double DEGREE_PER_TICK =  360 / 44032;

    public SwerveModule (TalonFX driveMotor, TalonFX turnMotor){
        this.Drive = driveMotor;
        
        this.Turn = turnMotor;

        Turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

        Turn.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
        Turn.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
        
        Turn.selectProfileSlot(0, 0);
        Turn.config_kF(0, 0.2, 30);
        Turn.config_kP(0, 0.1, 30);
        Turn.config_kI(0, 0, 30);
        Turn.config_kD(0, 0.01, 30);

        Turn.configMotionCruiseVelocity(30000, 30);
        Turn.configMotionAcceleration(12000, 30);
        
        Turn.configAllowableClosedloopError(0, 1024, 30);

        Turn.setSelectedSensorPosition(0, 0, 30);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(Drive.getSensorCollection().getIntegratedSensorVelocity(), new Rotation2d(((Turn.getSensorCollection().getIntegratedSensorPosition() * 44032) / 360) * (Math.PI / 180)));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        //Rotation2d r = new Rotation2d();
        //Rotation2d.fromDegrees((Turn.getSensorCollection().getIntegratedSensorPosition() * 44032) / 360) * (Math.PI / 180)));
        //SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(Turn.getSensorCollection().getIntegratedSensorPosition() * DEGREE_PER_TICK));
        
        SwerveModuleState state = desiredState;

        
        
        SmartDashboard.putNumber("Module Rotation", Rotation2d.fromDegrees(Turn.getSensorCollection().getIntegratedSensorPosition() * DEGREE_PER_TICK).getDegrees());
        SmartDashboard.putNumber("Module Speed", state.speedMetersPerSecond);

        Drive.set(ControlMode.PercentOutput, state.speedMetersPerSecond);

        SmartDashboard.putNumber("Module Angle Set", state.angle.getDegrees() * Constants.TURN_RATIO * TICKS_PER_DEGREE);

        Turn.set(ControlMode.MotionMagic, state.angle.getDegrees() * Constants.TURN_RATIO * TICKS_PER_DEGREE);

    }
}
