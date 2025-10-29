package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralHandlerConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.revrobotics.spark.SparkLowLevel.MotorType;

//Note Details on distance sensor driver installed from here: https://github.com/REVrobotics/2m-Distance-Sensor/?tab=readme-ov-file
@SuppressWarnings("unused")
public class AlgaeSubsystem extends SubsystemBase {
   private SparkMax motor;

//    public Rev2mDistanceSensor distanceSensor;
   public LEDSubsystem ledSubsystem;


    public AlgaeSubsystem() {
        // distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        motor = new SparkMax(AlgaeConstants.MOTOR_ID, MotorType.kBrushless);

        //Shuffleboard.getTab("Algae").addNumber("Algae Distance", () -> distanceSensor.GetRange());
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }
 

    //  public double getDistanceSensor() {
    //     if (distanceSensor.isRangeValid()) { 
    //         return distanceSensor.getRange();
    //     } else {
    //         return -1;
    //     }
    //  }

     private void setLEDColor(int[] color, String colorName) {
        // Placeholder for LED control
        ledSubsystem.changeLEDColor(color,colorName);
        System.out.println("LED Color: " + colorName);
    }

    public void stop(){
        /* 
        boolean intakeBroken // TO DO - ADD ALGAE LED COLORS
          if (!intakeBroken){
        setLEDColor(Constants.LEDConstants.GREEN, "green");
        } 
        */
        motor.stopMotor();
    }
}
