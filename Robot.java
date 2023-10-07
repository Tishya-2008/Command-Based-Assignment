// Import necessary libraries
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Define the RobotContainer class
public class RobotContainer {
    // The robot's subsystems
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    // The driver's controller
    Joystick driverController = new Joystick(0);
    Joystick operatorController = new Joystick(1);

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Create some buttons
        JoystickButton intakeInButton = new JoystickButton(operatorController, 1);
        JoystickButton intakeOutButton = new JoystickButton(operatorController, 2);

        // Connect the buttons to commands
        intakeInButton.whenPressed(new IntakeCommand(intakeSubsystem, 0.4));
        intakeOutButton.whenPressed(new IntakeCommand(intakeSubsystem, -0.4));
        
        // Add driver controls for arcadeDrive with axis 1 and 4
        driverController.arcadeDrive(driverController.getRawAxis(1), driverController.getRawAxis(4));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new ParallelCommandGroup(
            new DriveCommand(driveSubsystem, 0.5, 0),
            new IntakeCommand(intakeSubsystem, 0.4)
        ).withTimeout(2);
    }
}

// Define the DriveSubsystem class
public class DriveSubsystem extends SubsystemBase {
    // Motor controllers
    private final PWMVictorSPX frontLeft = new PWMVictorSPX(Constants.FRONT_LEFT_MOTOR);
    private final PWMVictorSPX frontRight = new PWMVictorSPX(Constants.FRONT_RIGHT_MOTOR);
    private final PWMVictorSPX backLeft = new PWMVictorSPX(Constants.BACK_LEFT_MOTOR);
    private final PWMVictorSPX backRight = new PWMVictorSPX(Constants.BACK_RIGHT_MOTOR);

    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

    public DriveSubsystem() {
        super();
        
        // Invert one of the sides of your drivetrain
        frontRight.setInverted(true);
        backRight.setInverted(true);
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(Math.min(fwd, Constants.FWD_MAX_SPEED), Math.min(rot, Constants.ROT_MAX_SPEED));
    }
}

// Define the IntakeSubsystem class
public class IntakeSubsystem extends SubsystemBase {
    // Motor controller
    private final PWMVictorSPX motor = new PWMVictorSPX(Constants.INTAKE_MOTOR);

    public IntakeSubsystem() {
        super();
        
        // Other requirements...
    }

    public void run(double speed) {
        motor.set(speed);
    }
}

// Define the DriveCommand class
public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final double forward;
    private final double rotation;

    public DriveCommand(DriveSubsystem subsystem, double fwd, double rot) {
        drive = subsystem;
        forward = fwd;
        rotation = rot;
        
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.arcadeDrive(forward, rotation);
    }
    
     @Override
     public boolean isFinished() {
         return false;
     }
     
     @Override
     public void end(boolean interrupted) {
         drive.arcadeDrive(0, 0);
     }
}

// Define the IntakeCommand class
public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double speed;

    public IntakeCommand(IntakeSubsystem subsystem, double spd) {
        intake = subsystem;
        speed = spd;
        
        addRequirements(intake);
    }

     @Override
     public void execute() {
         intake.run(speed);
     }
     
     @Override
     public boolean isFinished() {
         return false;
     }
     
     @Override
     public void end(boolean interrupted) {
         intake.run(Constants.STALL_SPEED);
     }
}