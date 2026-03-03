// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hopper extends SubsystemBase {


  // Create the possible states of the indexer
  public enum HopperState {
    IDLE,
    PASSIVE,
    HAWCK,
    TUHUA,
  }

  // Create a variable to store the current state of the indexer
  HopperState hopperState = HopperState.IDLE;

  // Keep track of whether or not the intake has a coral
  public boolean hasFuel = false;


  // Create a talonfx for the indexer
  TalonFX hopper = new TalonFX(Constants.Ports.HOPPER);


  // This CANrange is used to detect coral in the intake
  // CANrange INDEXERCANRANGE = new CANrange(Constants.Ports.INDEXERCANRANGE);


  // Indexer Constructor
  public Hopper() {

    // Create a talonfx configurator.
    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    // Inverted and Neutral Modes
    indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the indexer motor
    hopper.getConfigurator().apply(indexerConfig);

  }
    // Create a CANrange configurator

  /**
   * Get the current state of the indexer
   * 
   * @return the current state of the indexer as a IndexerState
   */
  public HopperState getState() {
    return hopperState;
  }
  
  /**
   * Set the state of the indexer
   * 
   * @param state the state to set the indexer to
   */
  public void setState(HopperState state) {
    hopperState = state;
  }

  /**
   * Set the speed of the indexer
   * <p>
   * Positive values will intake algae, negative values will intake coral
   * 
   * @param speed the speed to set the indexer to (-1 to 1)
   */
  

  public void sethopperSpeed(double speed) {
    hopper.set(speed);
  }

  

  /** 
   * Check to see whether the intake has either gamepiece
   * 
   * @return true if the intake has either gamepiece, false otherwise
   */
  public boolean hasGamepiece() {
    return hasFuel;
  }


  // public boolean isFuelIn() {
  //   return INDEXERCANRANGE.getDistance().getValueAsDouble() < 0.15 ;
  // }

  /**
   * Check to see if the indexer is aligned with the branch.
   * It does this by checking the distance detected by the branchCANRange.
   * If it detects an object closer than 0.5 meters, it is likely the branch, and thus we are aligned.
   * 
   * @return true if the indexer is aligned with the branch, false otherwise
   */
  




  @Override
  public void periodic() {
    // SmartDashboard.putNumber("CanRange Distance Detected", branchCANRange.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Indexer Current", hopper.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Indexer Status", String.valueOf(hopper.getSupplyCurrent().getValueAsDouble()));
    // SmartDashboard.putBoolean("Has Fuel", isFuelIn());
    



    // Indexer State Machine
    switch (hopperState) {

      // In the idle state, the indexer does not intake, and it isnt deactivated
      case IDLE:

      sethopperSpeed(0);
      

      break;




      case HAWCK:
        // if (isFuelIn() ) {
        //   Timer.delay(.20);
        //   setkickerSpeed(0);
        //   hasFuel = true;
        // }
        // else if (!hasGamepiece()) {
        //   setkickerSpeed(0.3);
        //   setindexerSpeed(.3);
        // }
        break;

      
      case TUHUA:

      // if(hopper.getSupplyCurrent().getValueAsDouble()> 20){

      //           sethopperSpeed(-1.0);
      // }
      // else
      //   sethopperSpeed(0.3);
      


        break;

      // In the passive state, the indexer will not intake, and will deactivate the intake. 
      //  This will be used when the indexer has a gamepiece
      case PASSIVE:

      if(hopper.getSupplyCurrent().getValueAsDouble()> 38)  {sethopperSpeed(.4);}

      else

        sethopperSpeed(-0.4);      
        

        break;
    }
  


    

  }
}