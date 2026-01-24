// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.TunerConstants;
import frc.robot.Constants;


public class Indexer extends SubsystemBase {


  // Create the possible states of the claw
  public enum IndexerState {
    IDLE,
    PASSIVE,
    HAWCK,
    TUHUA,
  }

  // Create a variable to store the current state of the claw
  IndexerState indexerState = IndexerState.IDLE;

  // Keep track of whether or not the intake has a coral
  public boolean hasFuel = false;


  // Create a talonfx for the claw
  TalonFX claw = new TalonFX(Constants.Ports.Indexer);

  // This CANrange is used to detect coral in the intake
  CANrange INDEXERCANRANGE = new CANrange(Constants.Ports.INDEXERCANRANGE);


  // Claw Constructor
  public Indexer() {

    // Create a talonfx configurator.
    TalonFXConfiguration IndexerConfig = new TalonFXConfiguration();

    // Inverted and Neutral Modes
    IndexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    IndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the claw motor
    claw.getConfigurator().apply(IndexerConfig);

  }
    // Create a CANrange configurator

  /**
   * Get the current state of the claw
   * 
   * @return the current state of the claw as a ClawState
   */
  public IndexerState getState() {
    return indexerState;
  }
  
  /**
   * Set the state of the claw
   * 
   * @param state the state to set the claw to
   */
  public void setState(IndexerState state) {
    indexerState = state;
  }

  /**
   * Set the speed of the claw
   * <p>
   * Positive values will intake algae, negative values will intake coral
   * 
   * @param speed the speed to set the claw to (-1 to 1)
   */
  public void setWheelSpeed(double speed) {
    claw.set(speed);
  }

  

  /** 
   * Check to see whether the intake has either gamepiece
   * 
   * @return true if the intake has either gamepiece, false otherwise
   */
  public boolean hasGamepiece() {
    return hasFuel;
  }


  public boolean isFuelIn() {
    return INDEXERCANRANGE.getDistance().getValueAsDouble() < 0.15 ;
  }

  /**
   * Check to see if the claw is aligned with the branch.
   * It does this by checking the distance detected by the branchCANRange.
   * If it detects an object closer than 0.5 meters, it is likely the branch, and thus we are aligned.
   * 
   * @return true if the claw is aligned with the branch, false otherwise
   */
  




  @Override
  public void periodic() {
    // SmartDashboard.putNumber("CanRange Distance Detected", branchCANRange.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Claw Current", claw.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Claw Status", String.valueOf(claw.getSupplyCurrent().getValueAsDouble()));
    SmartDashboard.putBoolean("Has Fuel", isFuelIn());
    



    // Claw State Machine
    switch (indexerState) {

      // In the idle state, the claw does not intake, and it isnt deactivated
      case IDLE:
        setWheelSpeed(0);
        hasFuel = false;
        break;





      // In the coral state, the claw will spin in reverse to intake coral,
      //  deactivating if the coralCANRange detects an object
      case HAWCK:
        if (isFuelIn() ) {
          Timer.delay(.20);
          setWheelSpeed(0);
          hasFuel = true;
        }
        else if (!hasGamepiece()) {
          setWheelSpeed(0.3);
        }
        break;

      // In the algae state, the claw will spin forwards to intake algae, 
      //  deactivating if the current exceeds 10 amps

      // In the release state, the claw will spin forwards to release the gamepiece
      //  and will release the deactivation lock
      case TUHUA:
        setWheelSpeed(0.3);
        hasFuel = false;
        break;

      // In the passive state, the claw will not intake, and will deactivate the intake. 
      //  This will be used when the claw has a gamepiece
      case PASSIVE:
        setWheelSpeed(-.3);
        

        break;
    }
  


    

  }
}