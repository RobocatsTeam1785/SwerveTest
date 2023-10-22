package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
public class SwerveModule {
    private final CANSparkMax e = new CANSparkMax(0, MotorType.kBrushless);
    public final double kWheelRadius = 0;
    public final double kDriveEncoderResolution = 0;//Pulses per Rotation
    public final double kTurnEncoderResolution = 0;//Pulses per Rotation
    public final double kModuleMaxAngularVelocity = 0;
    public final double kModuleMaxAngularAcceleration = 2*Math.PI;
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder driveEncoder;
    private Encoder turnEncoder;
    private final PIDController drivePIDController = new PIDController(0.3,0,0);//Not right numbers
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
        0.3,
        0,
        0,
        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)
    );
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1,3);//Parameters not right
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);//kS=volts, kV=volts * seconds / distance
    public SwerveModule(
        int driveMotorChannel,
        int turningMotorChannel,
        int turningEncoderChannelA,
        int turningEncoderChannelB){
            driveEncoder.setPosition(0);
            
            driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

            driveEncoder = driveMotor.getEncoder();
            turnEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

            driveEncoder.setPositionConversionFactor((2 * Math.PI * kWheelRadius)/kDriveEncoderResolution);
            driveEncoder.setVelocityConversionFactor((2 * Math.PI * kWheelRadius)/kDriveEncoderResolution);
            turnEncoder.setDistancePerPulse(2 * Math.PI / kTurnEncoderResolution);

            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getDistance()));
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turnEncoder.getDistance()));
    }
    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState state = 
            SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getDistance()));
        
        final double driveOutput = 
            drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double dFeedforward = 
            driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turnOutput = 
            turningPIDController.calculate(turnEncoder.getDistance(), state.angle.getRadians());
        
        final double tFeedforward = 
            turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);
        
        driveMotor.setVoltage(driveOutput + dFeedforward);
        turnMotor.setVoltage(turnOutput + tFeedforward);
    }


}
