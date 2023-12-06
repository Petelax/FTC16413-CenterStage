package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.commands.ArmPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.ClawCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrive
import org.firstinspires.ftc.teamcode.drive.commands.IntakeCommand
import org.firstinspires.ftc.teamcode.drive.commands.WristCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import java.util.function.DoubleSupplier

@TeleOp
class TeleOp: CommandOpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var elevator: Elevator
    private lateinit var arm: Arm
    private lateinit var intake: Intake
    private lateinit var claw: Claw
    private lateinit var wrist: Wrist
    private lateinit var driverOp: GamepadEx
    private lateinit var toolOp: GamepadEx
    private lateinit var x: DoubleSupplier
    private lateinit var elevatorSpeed: DoubleSupplier
    private lateinit var intakeSpeed: DoubleSupplier
    private var lastTime: Long = 0
    private var state: String = ""

    override fun initialize() {
        lastTime = System.nanoTime()
        drivebase = SwerveDrivebase(hardwareMap)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        arm = Arm(hardwareMap)
        intake = Intake(hardwareMap)
        claw = Claw(hardwareMap)
        wrist = Wrist(hardwareMap)
        driverOp = GamepadEx(gamepad1)
        toolOp = GamepadEx(gamepad2)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        x = DoubleSupplier { driverOp.leftX }
        elevatorSpeed = DoubleSupplier { toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) }
        intakeSpeed = DoubleSupplier { if(driverOp.getButton(GamepadKeys.Button.A)) { 1.0 } else { 0.0 } }


        drivebase.defaultCommand = FieldCentricDrive(
            drivebase,
            { driverOp.leftY * 1.0 },
            { driverOp.leftX * -1.0 },
            { driverOp.rightX * -1.0 },
            { false },
            { true }
        )

        elevator.defaultCommand = ElevatorCommand(
            elevator,
            elevatorSpeed
        )

        /*
        intake.defaultCommand = IntakeCommand(
            intake,
            intakeSpeed
        )
         */

        telemetry.addData("elevatorPos", elevator.getCurrentPosition())
        telemetry.addData("armPos", arm.getCurrentPosition())

        schedule(RunCommand(telemetry::update))

    }

    override fun run() {
        val currentTime = System.nanoTime()

        // arm
        if (elevator.getCurrentPosition() >= Elevator.ROTATE) {
            GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(
                ArmPIDCommand(
                    arm,
                    DrivebaseConstants.ArmPositions.TOP
                )
            )
            GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                ArmPIDCommand(
                    arm,
                    DrivebaseConstants.ArmPositions.BOTTOM
                )
            )
        }

        // claw
        GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN))
        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED))

        // intake
        if (elevator.getCurrentPosition() >= Elevator.INTAKE) {
            GamepadButton(toolOp, GamepadKeys.Button.A).whenHeld(IntakeCommand(intake, {1.0}))
            GamepadButton(toolOp, GamepadKeys.Button.B).whenHeld(IntakeCommand(intake, {-1.0}))
        } else {
            schedule(IntakeCommand(intake, {0.0}))
        }

        // weird elevator stuff
        GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(ElevatorPIDCommand(elevator, Elevator.INTAKE).withTimeout(10000))
        //GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(ElevatorCommand(elevator, elevatorSpeed).withTimeout(2000))
        GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(ElevatorPIDCommand(elevator, Elevator.TOP-1).withTimeout(10000))


        // wrist
        if (arm.getCurrentPosition() < DrivebaseConstants.ArmPositions.MIDDLE) {
            schedule(WristCommand(wrist, DrivebaseConstants.WristPositions.TOP))
        } else {
            schedule(WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM))
        }



        // automation attempt
        GamepadButton(toolOp, GamepadKeys.Button.START).whenPressed(
            SequentialCommandGroup(
                //InstantCommand({state = "CLAW + ELE"}),
                //ParallelCommandGroup(ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN), ElevatorPIDCommand(elevator, Elevator.BOTTOM+0.01)),
                InstantCommand({state = "CLAW OPEN"}),
                ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN).withTimeout(2000),
                InstantCommand({state = "ELE BOTTOM"}),
                ElevatorPIDCommand(elevator, Elevator.BOTTOM+0.01),
                InstantCommand({state = "CLAW CLOSED"}),
                ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED).withTimeout(5000),
                InstantCommand({state = "ELE UP"}),
                ElevatorPIDCommand(elevator, Elevator.ROTATE),
                InstantCommand({state = "ARM + ELE + WRIST"}),
                ParallelCommandGroup(
                    ElevatorPIDCommand(elevator, Elevator.TOP),
                    ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.TOP),
                    WristCommand(wrist, DrivebaseConstants.WristPositions.TOP)
                )
            ))

        /*
        GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(
            SequentialCommandGroup(
                ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN),
                ParallelCommandGroup(
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM),
                    ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                    WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)

                )
            )
        )

         */

        //GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(SequentialCommandGroup(ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED), ElevatorPIDCommand(elevator, Elevator.TOP-1)))

        // drivebase man
        GamepadButton(driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(InstantCommand(drivebase::resetGyro))
        GamepadButton(driverOp, GamepadKeys.Button.A).whenPressed(InstantCommand(drivebase::lock))

        telemetry.addLine(state)
        telemetry.addData("hz ", 1E9/(currentTime-lastTime))
        telemetry.addData("elevatorSpeed", elevator.get())
        telemetry.addData("elevatorPos", elevator.getCurrentPosition())
        telemetry.addData("elevatorSetpoint", elevator.setpoint)
        telemetry.addData("elevatorMode", elevator.mode)
        telemetry.addData("atSetPoint", elevator.atSetPoint())
        telemetry.addData("armPos", arm.getCurrentPosition())
        telemetry.addLine(drivebase.status())
        telemetry.update()

        lastTime = currentTime
        CommandScheduler.getInstance().run()
    }

}