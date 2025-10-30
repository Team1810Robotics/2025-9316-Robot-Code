package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;


public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotor2;

    private final Encoder elevatorEncoder;

    private PIDController elevatorPID;
    private ElevatorFeedforward elevatorFF;

    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.0);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);

    private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
    private LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
    private LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
    private LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);


    public double targetPosition = 0;
    public double totalPower = 0;
    public double elevatorVolts = 0.0;

    public double lastVelocity = 0.0;
    public double lastTime = 0.0;

    public ElevatorSubsystem() {
        // Initialize Motors
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);

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
        elevatorEncoder.setDistancePerPulse(1.0 / ElevatorConstants.TICKS_PER_INCH); 

        // Initialize PID Controller
        elevatorPID = new PIDController(kP.get(), kD.get(), kI.get()); // Adjust constants as needed
        elevatorPID.setTolerance(0.025); // Allowable error range

        // Set Feed Forward
        elevatorFF = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()); //TUNE

        Shuffleboard.getTab("Elevator").addNumber("Encoder Raw", () -> -elevatorEncoder.get());
        Shuffleboard.getTab("Elevator").addNumber("Elevator Height", () -> getPosition());
        Shuffleboard.getTab("Elevator").addNumber("Elevator Velocity", () -> getVeocity());

        Shuffleboard.getTab("Elevator").addNumber("Target Position", () -> targetPosition);
        Shuffleboard.getTab("Elevator").addNumber("Elevator Volts", () -> elevatorVolts);
        Shuffleboard.getTab("Elevator").add(elevatorPID);
    }

    /**
     * Set the target position of the elevator
     * @param target in meters
     */
    public void setElevatorPosition(double target) {
        targetPosition = target;

        if (isWithinBounds(targetPosition)) { 
            double pidVolts = elevatorPID.calculate(getPosition(), targetPosition);
            double ffVolts = elevatorFF.calculate(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);

            elevatorVolts = pidVolts + ffVolts;

            elevatorMotor.setVoltage(elevatorVolts);
        } else {
            System.out.println("Target position out of bounds: " + targetPosition);
        }
    }


    /**
     * 
     * @return the position of the elevator in meters
     */
    public double getPosition() {
        return Inches.of(elevatorEncoder.getDistance()).in(Meters);
    }

    /**
     * 
     * @return the velocity of the elevator in meters per second
     */
    public double getVeocity() {
        return InchesPerSecond.of(elevatorEncoder.getRate()).in(MetersPerSecond);
    }

    /**
     * 
     * @return the acceleration of the elevator in meters per second squared
     */
    public double getAcceleration() {
        double dt = Timer.getFPGATimestamp() - lastTime;

        double acceleration = (getVeocity() - lastVelocity) / dt;

        lastVelocity = getVeocity();
        lastTime = Timer.getFPGATimestamp();

        return acceleration;
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public boolean isWithinBounds(double position) {
        return position >= ElevatorConstants.MIN_HEIGHT && position <= ElevatorConstants.MAX_HEIGHT;
    }

    
    public void stop() {
        elevatorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (kS.hasChanged(hashCode()) || kG.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
            elevatorFF = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        }

        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            elevatorPID.setP(kP.get());
            elevatorPID.setI(kI.get());
            elevatorPID.setD(kD.get());
        }

    }
}
