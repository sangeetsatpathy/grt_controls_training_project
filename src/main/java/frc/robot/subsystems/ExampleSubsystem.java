// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.I2C;

public class ExampleSubsystem extends SubsystemBase {

  //definitions
  private final WPI_TalonSRX upperMotors;
  private final WPI_TalonSRX lowerMotor;
  private final XboxController controller;
  private final JoystickButton button;
  private final Timer timer;
  private final AnalogPotentiometer irSensorIntake;
  private final AnalogPotentiometer irSensorTop;
  private final ColorSensorV3 colorSensor;
  private final CANSparkMax flywheel;


  //constant variables
  private static final double FLYWHEEL_TIME = 0.5;
  private static final double FLYWHEEL_SPEED = 0.4;

  private static final double MOTOR_SPEED = -0.1;

  private static final double LOWER_IR_SENSOR_THRESHOLD = 0.26; //The threshold value for the lower IR sensor
  // this represents the value at which an IR sensor detects a ball

  private static final double TOP_IR_SENSOR_THRESHOLD = 0.30; // the threshold value for the higher IR sensor for when it detects a ball

  private static final double COLOR_SENSOR_THRESHOLD = 0.4;


  // boolean variables needed for code logic
  boolean intakeInProgress = false; // represents whether a new ball is being added in the shaft
  boolean ballDetectedByTopIR = false; // represents whether the top IR sensor has detected the ball for the first time
  boolean ballDetectedByColorSensor = false; // represents whether the color sensor has detected the ball for the first time
  boolean shotBtnPressed = false; //represents whether the controller has pressed the shoot button
  boolean shotTaken = false; // represents whether the last queued ball shot has been taken AND the repositioning has not happened yet

  int numBallsInShaft; // the number of balls at any given time IN THE SHAFT (not including the one at the intake).

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    upperMotors = new WPI_TalonSRX(14);
    upperMotors.configFactoryDefault();
    controller = new XboxController(0);
    button = new JoystickButton(controller, XboxController.Button.kA.value);

    lowerMotor = new WPI_TalonSRX(13);
    lowerMotor.configFactoryDefault();
    irSensorIntake = new AnalogPotentiometer(0);
    irSensorTop = new AnalogPotentiometer(1);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    flywheel = new CANSparkMax(15, MotorType.kBrushless);

    numBallsInShaft = 0;
    
    timer = new Timer();
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    
    if(intakeInProgress){ // if there are balls currently being inputted into the mechanism
      
		if(numBallsInShaft == 0){
			//if there are no balls in the shaft yet
			//loads the topmost ball all the way up to the flywheel such that the ball is above the top IR sensor.

			if(ballDetectedByTopIR){ // if the ball has been detected by the top IR sensor at least once before
			//this means that it is passing the sensor (or has passed it)

				if(irSensorTop.get() < TOP_IR_SENSOR_THRESHOLD){ // if the ir sensor does not detect the ball anymore, 
					//then the ball has gone just past the top ir sensor (where we want it). The motors STOP and variables updated accordingly.
					numBallsInShaft++;
					lowerMotor.set(0);
					upperMotors.set(0);

					//reset boolean variables for next action
					intakeInProgress = false;
					ballDetectedByTopIR = false;
				} else { // the ir sensor still detects the ball, so keep moving till the ball gets past the sensor.
					lowerMotor.set(MOTOR_SPEED);
					upperMotors.set(MOTOR_SPEED);
				}

			} else { //the ball has not been detected by the IR sensor yet
			//this means that the ball is under the IR sensor

				if(irSensorTop.get() >= TOP_IR_SENSOR_THRESHOLD){
					ballDetectedByTopIR = true; // the ball has been detected by the top IR, switch to the condition above.
				}
				lowerMotor.set(MOTOR_SPEED); 
				upperMotors.set(MOTOR_SPEED);

			}


		} else if(numBallsInShaft == 1){ // if there is already a ball at the top of the mechanism
			// this 2nd ball goes below the top IR sensor and just above the color sensor on the bottom, so we want to 
			// specifically just pass the color sensor.


				if(ballDetectedByColorSensor){ // if the color sensor has already detected the ball
				
					if(colorSensor.getProximity() < COLOR_SENSOR_THRESHOLD){
						//if the color sensor stops detecting the ball, then the ball is in its correct place, and the motors stop
						numBallsInShaft++;
						lowerMotor.set(0);
						upperMotors.set(0);

						//resets boolean variables for next action
						intakeInProgress = false;
						ballDetectedByColorSensor = false;
						
					} else {
						//if the color sensor still detects the ball, then keep moving the motor.
						lowerMotor.set(MOTOR_SPEED);
					}
				} else {
				// if the color sensor has not detected the ball yet

					if(colorSensor.getProximity() >= COLOR_SENSOR_THRESHOLD){ //if the color sensor detects the ball for the first time
						ballDetectedByColorSensor = true; // this switches to the first part of the conditional, which looks for the point at which
						//the ball first passes the conditional
					}
					lowerMotor.set(MOTOR_SPEED); 
				}
		} 


    } else if(shotBtnPressed){ // if there is no ball being intaken, and the button is pressed to shoot
      //the reason why this is an else-if is because the mechanism shouldn't shoot while the other motors are getting in place;
      // it could mess up the positioning

			//if the shot hasn't been taken yet, spin the flywheel for a certain amount of time.
			if(!shotTaken){
				if(timer.get() == 0){
					//if the timer hasn't started yet, start it (done only once).
					timer.start();
				}
				flywheel.set(FLYWHEEL_SPEED); // keep moving the motor until timer
				if(timer.hasElapsed(FLYWHEEL_TIME)){
					// if the motor has spun for enough time, stop the motor and reset the timer, breaking out of the timer loop
					shotTaken = true;
					timer.reset();
					flywheel.set(0);
					numBallsInShaft-=1;
				}
			} else { // the shot has been taken
			//the 2nd ball will now shift upward.
			
			/*NOTE: in the case that there were 2 balls in the shaft and one balloon in the intake, 
			once the first ball is shot and the 2nd ball is repositioned to the top, there will be 1 ball in the shaft (numBalls = 1).
			This means that the IR sensor at the intake can detect the third ball, and the "intakeInProgress" variable
			will automatically turn TRUE. This last ball will be treated as just another intake, so I don't need to add any 
			new code for it.
			*/

			//This is a repetition of the code for when we are inserting the first ball into the mechanism, except only the upper 
			// motors will be turning, since the 2nd ball only needs to move up one space
				if(ballDetectedByTopIR){
					if(irSensorTop.get() < TOP_IR_SENSOR_THRESHOLD){ // if the ir sensor does not detect the ball anymore, then the motor STOPS
					// At this point, the mechanism is done "reloading"
					intakeInProgress = false;
					lowerMotor.set(0);
					upperMotors.set(0);
					ballDetectedByTopIR = false;
					shotBtnPressed = false; 
					} else { // the ir sensor still detects the ball, keep moving motors
					lowerMotor.set(MOTOR_SPEED);
					upperMotors.set(MOTOR_SPEED);
					}
				} else { //the ball has not been detected by the IR sensor in the last cycle
		
					if(irSensorTop.get() >= TOP_IR_SENSOR_THRESHOLD){
					ballDetectedByTopIR = true; // the ball has been detected by the top IR (for the first time)
					}
					lowerMotor.set(MOTOR_SPEED);
					upperMotors.set(MOTOR_SPEED);
		
				}

			}

    } else { //if there is no ball currently being loaded up AND there is no shot that is currently queued
      /*if there is a shot currently queued to take place, the program will shoot and reposition the shaft BEFORE continuing
      to sense for balls to intake */
		if(irSensorIntake.get() >=  LOWER_IR_SENSOR_THRESHOLD && numBallsInShaft < 2){
			intakeInProgress = true;
		}

      //if there are 2 balls currently in the mechanism, the third ball does not move, it just stays at the intake.
      // therefore, there is no code necessary if there are 2 balls already in the shaft.
    }


    //if the controller button is pressed at any time, the program will note that the ball should be shot after any current intake
    // is being done.
    if(controller.getAButtonPressed()){
      shotBtnPressed = true;
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
