// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmHigh extends CommandBase  {

  private final ArmSubsystem m_armSubsystem;
  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final PIDController wristPIDController;

    /** Creates a new ArmHigh. */
    public ArmHigh(ArmSubsystem m_armSubsystem) {
          this.m_armSubsystem = m_armSubsystem;
          addRequirements(m_armSubsystem);

          this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKD); //Input the PID values for the shoulder
          shoulderPIDController.setTolerance(2); //Sets the tolerance for the sholder PID controller
          shoulderPIDController.setSetpoint(Arm.shoulderHighPosition); //Sets the setpoint for the shoulder PID controller

          this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
          elbowPIDController.setTolerance(2); //Sets the tolerance for the elbow PID controller
          elbowPIDController.setSetpoint(Arm.elbowHighPosition); //Sets the setpoint for the elbow PID controller

          this.wristPIDController = new PIDController(Arm.wristKP, Arm.wristKI, Arm.wristKD); //Input the PID values for the wrist
          wristPIDController.setTolerance(0.5); //Sets the tolerance for the wrist PID controller
          wristPIDController.setSetpoint(Arm.wristHighPosition); //Sets the setpoint for the wrist PID controller
    }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmHigh command started. Everyone stand back, I know what I'm doing. Trust me."); //Prints to the console
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle()); //Calculates the error for the shoulder
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle()); //Calculates the error for the elbow
    double wristSpeed = 1.5*wristPIDController.calculate(m_armSubsystem.getWristAngle()); //Calculates the error for the wrist

    m_armSubsystem.setSpeeds(shoulderSpeed, elbowSpeed, wristSpeed); //Sets the speeds for the elbow and wrist motors Shoulder speed set to 0 for testing purposes

    System.out.println("Shoulder Angle: " + m_armSubsystem.getShoulderAngle()); //Prints the shoulder angle to the console
    System.out.println("Elbow Angle: " + m_armSubsystem.getElbowAngle()); //Prints the elbow angle to the console
    System.out.println("Wrist Angle: " + m_armSubsystem.getWristAngle()); //Prints the wrist angle to the console

    SmartDashboard.putNumber("Shoulder Angle: ", m_armSubsystem.getShoulderAngle()); //Puts the shoulder angle on the SmartDashboard
    SmartDashboard.putNumber("Elbow Angle: ", m_armSubsystem.getElbowAngle()); //Puts the elbow angle on the SmartDashboard
    SmartDashboard.putNumber("Wrist Angle: ", m_armSubsystem.getWristAngle()); //Puts the wrist angle on the SmartDashboard
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopMotor(); //Stops the arm motors
    System.out.println("Arm Motors Stopped!!!!!"); //Prints to the console
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

