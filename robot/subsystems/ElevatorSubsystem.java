// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSubsystem extends SubsystemBase {
  
  private final TalonFX ElevatorMotor = new TalonFX(33);;
  public Angle position;
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0);

  public ElevatorSubsystem() {
    ElevatorMotor.setPosition( 0 );
    position = getEncoderPosition();
    
    // Let's name everything on the LiveWindow
    addChild("ElevatorMotor", ElevatorMotor);

    var ElevatorMotorConfiguration = new TalonFXConfiguration();
    ElevatorMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ElevatorMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ElevatorMotor.getConfigurator().apply( ElevatorMotorConfiguration );
    var talonFXConfigs1 = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs1 = talonFXConfigs1.Slot0;
    slot0Configs1.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs1.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs1.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs1.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs1.kI = 0; // no output for integrated error
    slot0Configs1.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs1 = talonFXConfigs1.MotionMagic;
    motionMagicConfigs1.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigs1.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs1.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

    ElevatorMotor.getConfigurator().apply(talonFXConfigs1);
    
  }

  

  public void movelittleup() {
    ElevatorMotor.setControl(m_request.withPosition(0.2));
    position = getEncoderPosition();
  }
  public void returnstartingposition() {
    ElevatorMotor.setControl(m_request.withPosition(0.0));
    position = getEncoderPosition();
  }

  public void IloveCheese() {
    ElevatorMotor.setControl(m_request.withPosition(0.98));
    position = getEncoderPosition();
  }

  public void itIsFourTwentyPMRightNow() {
    ElevatorMotor.setControl(m_request.withPosition(4.21));
  }

  public Angle getEncoderPosition() {
    var rotorPosSignal = ElevatorMotor.getRotorPosition();
    var rotorPos = rotorPosSignal.getValue();
    return rotorPos;
  }
  public void setEncoderPosition( Angle newposition) {
    position = newposition;
  }
  




  public void log() {
    // SmartDashboard.putData("ElevatorMotor", ElevatorMotor);
  }
  
  /** Call log method every loop. */
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
    log();
  }










}