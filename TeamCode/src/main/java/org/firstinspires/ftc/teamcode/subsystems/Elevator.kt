package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile

class Elevator(hardwareMap: HardwareMap, mode: Mode): SubsystemBase() {
    private var motorLeft: Motor
    private var motorRight: Motor

    private var elevator: MotorGroup
    private var feedforward: ElevatorFeedforward
    private var pid: ProfiledPIDController

    private var speed: Double = 0.0
    private var currentPosition: Double = 0.0
    private var rumble: Boolean = false
    private var atSetpoint: Boolean = false
    var setpoint = 0.0

    var mode: Mode


    init {
        this.mode = mode
        val pids = DrivebaseConstants.ElevatorPIDConstants
        motorLeft = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ELEVATOR_LEFT, Motor.GoBILDA.RPM_435)
        motorRight = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ELEVATOR_RIGHT, Motor.GoBILDA.RPM_435)

        motorLeft.stopAndResetEncoder()
        motorRight.stopAndResetEncoder()

        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        motorRight.inverted = true

        elevator = MotorGroup(motorLeft, motorRight)
        feedforward = ElevatorFeedforward(pids.kS, pids.kG, pids.kV, pids.kA)
        pid = ProfiledPIDController(pids.kP, pids.kI, pids.kD, TrapezoidProfile.Constraints(pids.MAX_VELO, pids.MAX_ACCEL))
        pid.setTolerance(pids.POSITION_TOLERANCE, pids.VELOCITY_TOLERANCE)

    }

    enum class Mode {
        PID, RAW, IDLE
    }

    /*
    120mm per rotation
    696mm total travel
     */
    fun ticksToInches(ticks: Double): Double {
        return ticks / 5.8
    }

    fun resetEncoder() {
        motorRight.resetEncoder()
        motorLeft.resetEncoder()
    }

    fun set(speed: Double) {
        currentPosition = getCurrentPosition()
        val v = speed

        if ((currentPosition < BOTTOM && speed < 0.0) || (currentPosition > TOP && speed > 0.0)) {
            elevator.set(0.0)
        } else {
            elevator.set(v + DrivebaseConstants.ElevatorPIDConstants.kG)
        }

        this.speed = speed
    }

    fun setSetPoint(setpoint: Double) {
        currentPosition = getCurrentPosition()
        this.setpoint = setpoint
        pid.setGoal(setpoint)
        val pidCalc = pid.calculate(currentPosition, setpoint)
        var v = 0.0
        if (!pid.atSetpoint()) {
            v = pidCalc
        }

        if ((currentPosition < BOTTOM && speed < 0.0) || (currentPosition > TOP && speed > 0.0)) {
            elevator.set(0.0)
        } else {
            elevator.set(v + DrivebaseConstants.ElevatorPIDConstants.kG)
        }

    }

    fun getRumble(): Boolean {
        return rumble
    }

    fun get(): Double {
        return elevator.get()
    }

    fun getCurrentPositionTicks(): Double {
        return currentPosition
    }

    fun getCurrentPosition(): Double {
        return ((motorLeft.currentPosition + motorRight.currentPosition).toDouble() / 2) / DrivebaseConstants.Measurements.ELEVATOR_TICKS_PER_INCH
    }

    fun getAll(): List<Int> {
        return listOf(motorLeft.currentPosition, motorRight.currentPosition)
    }

    fun atSetPoint(): Boolean {
        pid.setTolerance(DrivebaseConstants.ElevatorPIDConstants.POSITION_TOLERANCE, DrivebaseConstants.ElevatorPIDConstants.VELOCITY_TOLERANCE)
        return pid.atSetpoint()
    }

    fun atBottomSetPoint(): Boolean {
        pid.setTolerance(4.0, DrivebaseConstants.ElevatorPIDConstants.VELOCITY_TOLERANCE)
        return pid.atSetpoint()
    }

    fun velocityControl(n: Boolean) {
        val pids = DrivebaseConstants.ElevatorPIDConstants
        if (n) {
            pid.setTolerance(pids.POSITION_TOLERANCE, pids.VELOCITY_TOLERANCE)
        } else {
            pid.setTolerance(pids.POSITION_TOLERANCE, 1000000.0)
        }
    }


    override fun periodic() {
        currentPosition = getCurrentPosition()
        /*
        pid.setGoal(setpoint)
        val pidCalc = pid.calculate(currentPosition, setpoint)

        speed = when(mode) {
            Mode.RAW -> { speed }
            Mode.PID -> {
                if (pid.atSetpoint()) {
                    0.0
                } else {
                    pidCalc
                }
            }
            Mode.IDLE -> { 0.0 }
            else -> { throw Exception("idk how, but ur bad") }
        }

         */

        /*
        if ((currentPosition < BOTTOM && speed < 0.0) || (currentPosition > TOP && speed > 0.0)) {
            elevator.set(0.0)
        } else {
            elevator.set(speed + DrivebaseConstants.ElevatorPIDConstants.kG)
        }

         */

        /*
        if (speed != 0.0) {
            elevator.set(speed + DrivebaseConstants.ElevatorPIDConstants.kG)
        } else {
            elevator.set(0.0)
        }

         */


    }

    companion object {
        const val TOP = 26.0 //2000.0
        const val BOTTOM = 0.01
        const val INTAKE = 4.0 //136.0
        const val ROTATE = 6.0
        const val MIDDLE_PLACE = 18.0
    }


}