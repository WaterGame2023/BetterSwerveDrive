// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax.IdleMode;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {

 private TalonFX shoulderFalcon = new TalonFX(Arm.shoulderMotorID);
 private CANSparkMax elbowMotor = new CANSparkMax(Arm.elbowMotorID, MotorType.kBrushless);
 private CANSparkMax wristMotor = new CANSparkMax(Arm.wristMotorID, MotorType.kBrushless);
 private TalonFX clawFalcon = new TalonFX(Arm.clawMotorID);

 private double WristSpeed = 0;

//Set neuteral mode to brake for each motor
public final void init() {
  shoulderFalcon.setNeutralMode(NeutralMode.Brake);
  elbowMotor.setIdleMode(IdleMode.kBrake);
  wristMotor.setIdleMode(IdleMode.kBrake);
  clawFalcon.setNeutralMode(NeutralMode.Brake);
  zeroAllEncoders();

  //Set open and closed loop ramp rates
  shoulderFalcon.configOpenloopRamp(0);
  shoulderFalcon.configClosedloopRamp(0);
  elbowMotor.setOpenLoopRampRate(0);
  elbowMotor.setClosedLoopRampRate(0);
  wristMotor.setOpenLoopRampRate(0);
  wristMotor.setClosedLoopRampRate(0);
  clawFalcon.configOpenloopRamp(0);
  clawFalcon.configClosedloopRamp(0);

  //Set the encoder values to read every 30 ms
  elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
  wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);

}

  public void setSpeeds(double shoulderSpeed, double elbowSpeed, double wristSpeed,double clawSpeed) {
    this.WristSpeed = wristSpeed;
    shoulderFalcon.set(ControlMode.PercentOutput, shoulderSpeed);    
    elbowMotor.set(elbowSpeed);
    wristMotor.set(wristSpeed);
    clawFalcon.set(ControlMode.PercentOutput, clawSpeed);

  }

public void stopMotor() {
  shoulderFalcon.set(ControlMode.PercentOutput, 0);
  elbowMotor.set(0);
  wristMotor.set(0);
  clawFalcon.set(ControlMode.PercentOutput, 0);
}

public double getShoulderAngle() {
  return shoulderFalcon.getSelectedSensorPosition() / 125;
}

public double getElbowAngle() {
  return elbowMotor.getEncoder().getPosition() / 125;
}

public double getWristAngle() {
  return wristMotor.getEncoder().getPosition() / 125;
}

public double getClawPosition() {
  return clawFalcon.getSelectedSensorPosition();
}

public double getShoulderDegrees() {
  return shoulderFalcon.getSelectedSensorPosition() * 360 / 4096; //TODO Compensate for gearbox ratio
}

public double getElbowDegrees() {
  return elbowMotor.getEncoder().getPosition() * 360 / 42; //TODO Compensate for gearbox ratio
}

public double getWristDegrees() {
  return wristMotor.getEncoder().getPosition() * 360 / 42; //TODO Compensate for gearbox ratio
}

public void zeroAllEncoders() {
  shoulderFalcon.setSelectedSensorPosition(0);
  elbowMotor.getEncoder().setPosition(0);
  wristMotor.getEncoder().setPosition(0);
  clawFalcon.getSelectedSensorPosition(0);
  System.out.println("Arm Encoders Zeroed");
}

public void releaseAllMotors() {
shoulderFalcon.setNeutralMode(NeutralMode.Coast);
elbowMotor.setIdleMode(IdleMode.kCoast);
wristMotor.setIdleMode(IdleMode.kCoast);
clawFalcon.setNeutralMode(NeutralMode.Coast);
System.out.println("Motors Released!");
}

public void brakeAllMotors() {
  shoulderFalcon.setNeutralMode(NeutralMode.Brake);
  elbowMotor.setIdleMode(IdleMode.kBrake);
  wristMotor.setIdleMode(IdleMode.kBrake);
  clawFalcon.setNeutralMode(NeutralMode.Brake);
  System.out.println("Motors Braked!");
  }

@Override
public void periodic(){
  SmartDashboard.putNumber("Shoulder Position: ", getShoulderAngle());
  SmartDashboard.putNumber("Elbow Position: ", getElbowAngle());
  SmartDashboard.putNumber("Wrist Position: ", getWristAngle());
  SmartDashboard.putNumber("Claw Position: ", getClawPosition());
  SmartDashboard.putNumber("Wrist Speed: ", WristSpeed);
}

}