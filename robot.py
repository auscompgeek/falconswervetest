import math

import ctre
import wpilib
from rev import CANSparkMax, MotorType, ControlType
from wpimath.kinematics import SwerveDrive2Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d

from utilities.scalers import rescale_js, scale_value
from utilities.functions import constrain_angle
from wpimath.controller import SimpleMotorFeedforwardMeters


class SwerveModule:
    DRIVE_GEAR_RATIO = 16 / 60
    STEER_GEAR_RATIO = 1 / 60

    DRIVE_SENSOR_TO_METRES = DRIVE_GEAR_RATIO / 2048
    METRES_TO_DRIVE_UNITS = 2048 / DRIVE_GEAR_RATIO

    def __init__(self, x: float, y: float, steer: CANSparkMax, drive: ctre.WPI_TalonFX):
        self.translation = Translation2d(x, y)

        self.steer = steer
        self.encoder = self.steer.getAnalog()
        self.hall_effect = self.steer.getEncoder()
        # make the sensor's return value between 0 and 1
        self.encoder.setPositionConversionFactor(math.tau/3.3)
        self.encoder.setInverted(True)
        self.hall_effect.setPositionConversionFactor(self.STEER_GEAR_RATIO * math.tau)
        self.rezero_hall_effect()
        self.steer_pid = steer.getPIDController()
        self.steer_pid.setFeedbackDevice(self.hall_effect)
        self.steer_pid.setP(1.85e-5)
        self.steer_pid.setD(0)
        self.steer_pid.setFF(0.583 / 12)
        self.steer_pid.setSmartMotionMaxVelocity(10)  # RPM
        self.steer_pid.setSmartMotionMaxAccel(10)  # RPM/s

        self.drive = drive
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.757, kV=1.3, kA=0.0672)

        drive.config_kP(slotIdx=0, value=2.21e-31, timeoutMs=20)
    
    def get_angle(self) -> Rotation2d:
        return Rotation2d(self.hall_effect.getPosition())

    def get_speed(self):
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_SENSOR_TO_METRES * 10
    
    def set(self, desired_state: SwerveModuleState):
        self.steer_pid.setReference(desired_state.angle.radians(), ControlType.kSmartMotion)
        # rescale the speed target based on how close we are to being correctly aligned
        error = self.get_angle() - desired_state.angle
        speed_target = desired_state.speed * error.cos()
        speed_volt = self.drive_ff.calculate(speed_target)
        voltage = wpilib.RobotController.getInputVoltage()
        self.drive.set(
            ctre.ControlMode.Velocity,
            speed_target * self.METRES_TO_DRIVE_UNITS / 10,
            ctre.DemandType.ArbitraryFeedForward,
            speed_volt / voltage,
        )

    def rezero_hall_effect(self):
        self.hall_effect.setPosition(self.encoder.getPosition())


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        self.modules = [
            SwerveModule(0.8/2-0.125, 0.75/2-0.1, CANSparkMax(9, MotorType.kBrushless), ctre.WPI_TalonFX(3)),
            SwerveModule(-0.8/2-0.125, -0.75/2-0.1, CANSparkMax(7, MotorType.kBrushless), ctre.WPI_TalonFX(5))
        ]

        self.kinematics = SwerveDrive2Kinematics(self.modules[0].translation, self.modules[1].translation)

        self.joystick = wpilib.Joystick(0)

        self.spin_rate = 1.5

    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5
        ) * 4 * throttle
        joystick_vy = -rescale_js(
            self.joystick.getX(), deadzone=0.1, exponential=1.5
        ) * 4 * throttle
        joystick_vz = -rescale_js(
            self.joystick.getZ(), deadzone=0.2, exponential=20.0
        ) * self.spin_rate
        chassis_speeds = ChassisSpeeds(joystick_vx, joystick_vy, joystick_vz)

        
        for state, module in zip(self.kinematics.toSwerveModuleStates(chassis_speeds), self.modules):
            state = SwerveModuleState.optimize(state, module.get_angle())
            module.set(state)

if __name__ == "__main__":
    wpilib.run(MyRobot)
