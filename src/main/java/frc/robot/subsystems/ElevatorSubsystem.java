package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.ElevatorFeedforward;


public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotor2;
    private final Encoder elevatorEncoder;
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFF;
    private static final double MAX_HEIGHT = 40;
    private static final double MIN_HEIGHT = 0;
    public static final double INTAKE_POSITION = 0;
    public static final double L1_POSITION = 10.25;        //11    
    public static final double L2_POSITION = 26;        //27
    public static final double Algae1 = 18.5;        //36
    public static final double Algae2 = 34.5;        //36
    public static final double climb = 40;        //36
    public static final double MANUAL_ADJUST_INCREMENT = .5; // Small adjustment for manual control
    public static double targetPosition = 0;
    public static double totalPower = 0;
        private static final double TICKS_PER_INCH = 185.0;
    
        public LEDSubsystem ledSubsystem;

        private double lastRawValue = 0.0;
        private int rotationCount = 0;
        public double pidOutput =0; 
    
        private static double elevatorPower;
    
        public final CoralHandlerSubsystem coralHandler;

        //private final DigitalInput intakeBeamBreak = new DigitalInput(CoralHandlerConstants.INTAKE_BEAM_BREAK_ID);
    
        public ElevatorSubsystem(CoralHandlerSubsystem coralHandler, LEDSubsystem ledSubsystem) {
            this.coralHandler = coralHandler;
            this.ledSubsystem = ledSubsystem;
            // Initialize Motors
            elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
            elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
    
            // Set Feed Forward
            elevatorFF = new ElevatorFeedforward(0.0, 0.3, 0.0); //TUNE

            // Configure Motors
            SparkMaxConfig config_ = new SparkMaxConfig();
            SparkMaxConfig config_2 = new SparkMaxConfig();
    
            config_.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit);
            config_2.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit)
                    .follow(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, true); // Inverted follow
    
            elevatorMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
            elevatorMotor2.configure(config_2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
            // Initialize Encoder
            elevatorEncoder = new Encoder(8, 9);
            elevatorEncoder.setDistancePerPulse(1.0 / TICKS_PER_INCH); 
    
            // Initialize PID Controller
            elevatorPID = new PIDController(0.225, 0.001, 0.01); // Adjust constants as needed
            elevatorPID.setTolerance(0.025); // Allowable error range

            Shuffleboard.getTab("Elevator").addNumber("Encoder Raw", () -> -elevatorEncoder.get());
            Shuffleboard.getTab("Elevator").addNumber("Elevator Height", () -> getElevatorPosition());

            Shuffleboard.getTab("Elevator").addNumber("Target Position", () -> targetPosition);

        }
    
        // Setpoint method for PID control
        public void setElevatorPosition(double Target) {
            targetPosition = Target;
    
            if (isWithinBounds(targetPosition)) { 
                double pidOutput = elevatorPID.calculate(getElevatorPosition(), targetPosition);
                // double ffOutput = elevatorFF.calculate();
            }
        }

    
        public void stop() {
            elevatorMotor.set(0);
        }

        public double getVeocity() {
            return elevatorEncoder.getRate() / TICKS_PER_INCH;
        }
    
        public double getElevatorPosition() {
            return elevatorEncoder.getDistance();
        }

    public double getTargetPosition(){
        return targetPosition;
    }

    public double getElevatorPower(){
        return elevatorPower;
    }

    public boolean isWithinBounds(double position) {
        return position >= MIN_HEIGHT && position <= MAX_HEIGHT;
    }

    private void setLEDColor(int[] color, String colorName) {
        // Placeholder for LED control
        ledSubsystem.changeLEDColor(color, colorName);
        System.out.println("LED Color: " + colorName);
    }

    @Override
    public void periodic() {

        
    }
}
