import math

import ctre
import wpilib
from rev import CANSparkMax, MotorType, ControlType
from wpimath.kinematics import SwerveDrive2Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d

from utilities.functions import constrain_angle, rescale_js


class SwerveModule:

    GEAR_RATIO = 1 / 60

    def __init__(self, x: float, y: float, steer: CANSparkMax, drive: ctre.WPI_TalonFX):
        self.translation = Translation2d(x, y)

        self.steer = steer
        self.encoder = self.steer.getAnalog()
        self.hall_effect = self.steer.getEncoder()
        # make the sensor's return value between 0 and 1
        self.encoder.setPositionConversionFactor(math.tau/3.3)
        self.hall_effect.setPositionConversionFactor(self.GEAR_RATIO * math.tau)
        self.hall_effect.setPosition(self.encoder.getPosition())
        self.steer_pid = steer.getPIDController()
        self.steer_pid.setFeedbackDevice(self.hall_effect)

        self.drive = drive
    
    def get_angle(self):
        return self.hall_effect.getPosition()

    def get_speed(self):
        return self.drive.getSelectedSensorVelocity()
    
    def set(self, desired_state: SwerveModuleState):
        self.steer_pid.setReference(desired_state.angle, ControlType.kSmartMotion)
        # rescale the speed target based on how close we are to being correctly aligned
        speed_target = desired_state.speed * math.cos(self.steer_pid.getLastError())
        drive.set(ctre.ControlMode.MotionMagic, speed_target)

    def rezero_hall_effect(self):
        self.hall_effect.setPosition(self.encoder.getPosition())


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        self.modules = [
            SwerveModule(1, 1, CANSparkMax(4, MotorType.kBrushless), ctre.WPI_TalonFX(3)),
            SwerveModule(-1, -1, CANSparkMax(6, MotorType.kBrushless), ctre.WPI_TalonFX(5))
        ]

        self.kinematics = SwerveDrive2Kinematics(self.modules[0].translation, self.modules[1].translation)


    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        throttle = max(0.1, (1 - self.joystick.getThrottle()) / 2)  # min 10%

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vy = -rescale_js(
            self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vz = -rescale_js(
            self.joystick.getZ(), deadzone=0.2, exponential=20.0, rate=self.spin_rate
        )
        if joystick_vx or joystick_vy or joystick_vz:
            chassis_speeds = ChassisSpeeds(joystick_vx, joystick_vy, joystick_vz)
        else:
            chassis_speeds = ChassisSpeeds(0, 0, 0)

        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        for i, module_state in enumerate(module_states):
            angle = Rotation2d(self.modules[i].get_angle())
            module_states[i] = SwerveModuleState.optimize(module_state, angle)



if __name__ == "__main__":
    wpilib.run(MyRobot)