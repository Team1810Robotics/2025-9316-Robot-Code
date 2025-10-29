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

@SuppressWarnings("unused")
public class AlgaeSubsystem extends SubsystemBase {
   private SparkMax motor;

   public LEDSubsystem ledSubsystem;


    public AlgaeSubsystem() {
        motor = new SparkMax(AlgaeConstants.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

     private void setLEDColor(int[] color, String colorName) {
        // Placeholder for LED control
        ledSubsystem.changeLEDColor(color,colorName);
        System.out.println("LED Color: " + colorName);
    }

    public void stop(){
        motor.stopMotor();
    }
}
