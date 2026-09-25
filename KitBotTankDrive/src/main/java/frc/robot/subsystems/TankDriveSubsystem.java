// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDriveSubsystem extends SubsystemBase {

  private SparkMax left0;
  private SparkMax left1;
  private SparkMax right0;
  private SparkMax right1;

  private double leftPower;
  private double rightPower;

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem() {
    left0 = new SparkMax(1, MotorType.kBrushed);
    left1 = new SparkMax(2, MotorType.kBrushed);
    right0 = new SparkMax(3, MotorType.kBrushed);
    right1 = new SparkMax(4, MotorType.kBrushed);

    SparkMaxConfig left0Config = new SparkMaxConfig();
    SparkMaxConfig left1Config = new SparkMaxConfig();
    SparkMaxConfig right0Config = new SparkMaxConfig();
    SparkMaxConfig right1Config = new SparkMaxConfig();

    left0Config.inverted(false);
    left1Config.inverted(false);
    right0Config.inverted(true);
    right1Config.inverted(true);

    left1Config.follow(left0);
    right1Config.follow(right0);

    left0.configure(left0Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    left1.configure(left1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    right0.configure(right0Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    right1.configure(right1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void tankDrive(double leftStick, double rightStick) {
     if(leftStick< Constants.kDeadBand && leftStick > -Constants.kDeadBand) {
        leftStick = 0; 

    }
    if(rightStick< Constants.kDeadBand && rightStick> -Constants.kDeadBand){ 
      rightStick=0;
    }


    leftPower = leftStick *leftStick;
    rightPower = rightStick *rightStick;
  
    if(leftStick < 0){
      leftPower=-leftPower;
    }
    if(rightStick < 0){
      rightPower = -rightPower;
    }
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    left0.set(leftPower);
    right0.set(rightPower);
  }
}
