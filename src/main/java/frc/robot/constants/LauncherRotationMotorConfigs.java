package frc.robot.constants;


import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
// import com.ctre

import frc.robot.constants.Constants.LauncherConstants;

public final class LauncherRotationMotorConfigs{

    public TalonSRXConfiguration motorConfig;
    
    public LauncherRotationMotorConfigs(){
      motorConfig = new TalonSRXConfiguration();
      motorConfig.slot0.kP = LauncherConstants.kP;
      motorConfig.slot0.kI = LauncherConstants.kI;
      motorConfig.slot0.kD = LauncherConstants.kD;
      motorConfig.slot0.kF = LauncherConstants.kF;
    
      motorConfig.continuousCurrentLimit = LauncherConstants.motorContinuousCurrentLimit;
      motorConfig.peakCurrentDuration = LauncherConstants.motorPeakCurrentDuration;
      motorConfig.peakCurrentLimit = LauncherConstants.motorPeakCurrentLimit;
      motorConfig.openloopRamp = LauncherConstants.motorOpenloopRamp;
      motorConfig.closedloopRamp = LauncherConstants.motorClosedloopRamp;
      motorConfig.motionAcceleration = 20.4;
      motorConfig.motionCruiseVelocity = 24.8;
      motorConfig.motionCurveStrength = 9;
    }
  }
