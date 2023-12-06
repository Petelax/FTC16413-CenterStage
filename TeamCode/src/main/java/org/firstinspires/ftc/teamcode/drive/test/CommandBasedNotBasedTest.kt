package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.commands.ArmCommand
import org.firstinspires.ftc.teamcode.drive.commands.ArmPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.ClawCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorNoVeloCommand
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
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import java.util.function.DoubleSupplier

@TeleOp
class CommandBasedNotBasedTest: CommandOpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var intake: Intake
    private lateinit var elevator: Elevator
    private lateinit var wrist: Wrist
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var driverOp: GamepadEx
    private lateinit var toolOp: GamepadEx
    private var state: State = State.BOTTOM
    private var lastTime: Long = 0
    private var intakeSpeed: Double = 0.0
    private var fieldCentric = true
    private lateinit var voltage: VoltageSensor

    private var topx = 0.0
    private var topy = 0.0
    private var topheading = 0.0

    enum class State {
        BOTTOM,
        INTAKE,
        PLACING
    }

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drivebase = SwerveDrivebase(hardwareMap)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        wrist = Wrist(hardwareMap)
        claw = Claw(hardwareMap)
        arm = Arm(hardwareMap)
        intake = Intake(hardwareMap)
        driverOp = GamepadEx(gamepad1)
        toolOp = GamepadEx(gamepad2)
        voltage = hardwareMap.get(VoltageSensor::class.java, "Control Hub")
        schedule(WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM))
        /*schedule(
            InstantCommand( { claw.setGripper(DrivebaseConstants.GrabberPositions.CLOSED) } ),
        )
         */

        drivebase.defaultCommand = FieldCentricDrive(
            drivebase,
            {driverOp.leftY},
            {driverOp.leftX*-1.0},
            {driverOp.rightX*-1.0},
            {driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)},
            {fieldCentric}
        )
        elevator.defaultCommand = ElevatorCommand(elevator) { toolOp.leftY }
        arm.defaultCommand = ArmCommand(arm) {toolOp.rightY}
        intake.defaultCommand = IntakeCommand(intake) { intakeSpeed }
    }

    override fun run() {
        val currentTime = System.nanoTime()

        /*
        schedule(InstantCommand({elevator.set(
            gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(
                GamepadKeys.Trigger.LEFT_TRIGGER
            )
        )})
        )
         */

        // Top placing
        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(
            ConditionalCommand(
                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN).withTimeout(2000),
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM+0.01),
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED).withTimeout(5000),
                    WaitCommand(500),
                    ElevatorNoVeloCommand(elevator, Elevator.ROTATE),
                    //ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.MIDDLE),
                    ParallelCommandGroup(
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.TOP),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.TOP)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.TOP),
                    //ElevatorPIDCommand(elevator, Elevator.TOP),
                    InstantCommand({state = State.PLACING})
                ),
                SequentialCommandGroup(
                    ElevatorPIDCommand(elevator, Elevator.TOP),
                )
            ) {arm.getCurrentPosition() >= DrivebaseConstants.ArmPositions.MIDDLE}
        )

        // Middle
        GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(
            ConditionalCommand(
                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN).withTimeout(1000),
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM+0.0),
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED).withTimeout(5000),
                    WaitCommand(500),
                    ElevatorPIDCommand(elevator, Elevator.ROTATE),
                    //ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.MIDDLE),
                    ParallelCommandGroup(
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.TOP),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.TOP)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.MIDDLE_PLACE),
                    //ElevatorPIDCommand(elevator, Elevator.TOP),
                    InstantCommand({state = State.PLACING})
                ),
                SequentialCommandGroup(
                    ElevatorPIDCommand(elevator, Elevator.MIDDLE_PLACE),
                )
            ) {arm.getCurrentPosition() >= DrivebaseConstants.ArmPositions.MIDDLE}
        )

        // Intake Height
        GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(
            ConditionalCommand(
                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN),
                    WaitCommand(300),
                    ParallelCommandGroup(
                        ElevatorPIDCommand(elevator, Elevator.ROTATE),
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.INTAKE),
                    InstantCommand({state = State.INTAKE})
                ),
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.INTAKE),
                    InstantCommand({state = State.INTAKE})
                )
            ) {arm.getCurrentPosition() <= DrivebaseConstants.ArmPositions.MIDDLE}

        )

        // Bottom
        GamepadButton(toolOp, GamepadKeys.Button.A).whenPressed(
            ConditionalCommand(
                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN),
                    WaitCommand(150),
                    ParallelCommandGroup(
                        ElevatorPIDCommand(elevator, Elevator.INTAKE),
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM),
                    InstantCommand({state = State.BOTTOM})
                ),
                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN),
                    WaitCommand(150),
                    ParallelCommandGroup(
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM),
                    InstantCommand({state = State.BOTTOM})
                )
            ) {arm.getCurrentPosition() <= DrivebaseConstants.ArmPositions.MIDDLE}
        )

        // Intake
        intakeSpeed = toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        if (MathUtil.applyDeadband(intakeSpeed, 0.001) != 0.0) {
            schedule(ElevatorPIDCommand(elevator, Elevator.INTAKE))
        }

        // Field Centric
        GamepadButton(driverOp, GamepadKeys.Button.A).whenPressed(
            InstantCommand({fieldCentric = true})
        )
        GamepadButton(driverOp, GamepadKeys.Button.B).whenPressed(
            InstantCommand({fieldCentric = false})
        )

        // Claw
        GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            ConditionalCommand(
                InstantCommand({claw.setRight(DrivebaseConstants.GrabberPositions.OPEN)}),
                SequentialCommandGroup()
            ) {arm.getCurrentPosition() <= DrivebaseConstants.ArmPositions.MIDDLE}
        )
        GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            ConditionalCommand(
                InstantCommand({claw.setLeft(DrivebaseConstants.GrabberPositions.OPEN)}),
                SequentialCommandGroup()
            ) {arm.getCurrentPosition() <= DrivebaseConstants.ArmPositions.MIDDLE}
        )

        // Wrist
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(
            WristCommand(wrist, DrivebaseConstants.WristPositions.TOP)
        )
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(
            WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
        )

        // Reset Elevator
        GamepadButton(toolOp, GamepadKeys.Button.BACK).whenPressed(
            InstantCommand(elevator::resetEncoder)
        )

        // Reset Gyro
        GamepadButton(driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(InstantCommand(drivebase::resetGyro))

        // Climb
        GamepadButton(driverOp, GamepadKeys.Button.Y).whenHeld(InstantCommand({elevator.set(-100.0)}))

        val poseVelocity = drivebase.getPoseVelocity()
        val pose = drivebase.getPose()
        //val packet = TelemetryPacket()
        //packet.fieldOverlay().setFill("blue").fillRect(pose.x-7.0, pose.y-7.0, 14.0, 14.0)
        /*
        packet.put("hz ", 1E9/(currentTime-lastTime))
        packet.put("linVelX", poseVelocity.linearVel.x)
        packet.put("linVelY", poseVelocity.linearVel.y)
        packet.put("angVel", poseVelocity.angVel)
        packet.put("state", state)
        packet.put("armPos", arm.getCurrentPosition())
        packet.put("armSpeed", arm.get())
        packet.put("claw", claw.getGrabber())
        packet.put("elevatorSpeed", elevator.get())
        packet.put("elevatorPos", elevator.getCurrentPosition())
        packet.put("elevatorSetpoint", elevator.setpoint)
        packet.put("elevatorMode", elevator.mode)
        packet.put("atSetPoint", elevator.atSetPoint())
        
         */

        topx = MathUtil.max(topx, poseVelocity.linearVel.x)
        topy = MathUtil.max(topy, poseVelocity.linearVel.y)
        topheading = MathUtil.max(topheading, poseVelocity.angVel)

        telemetry.addData("hz ", 1E9/(currentTime-lastTime))
        telemetry.addData("voltage", voltage.voltage)
        telemetry.addData("accel x", drivebase.getPoseAccel().linearVel.x)
        telemetry.addData("accel y", drivebase.getPoseAccel().linearVel.y)
        telemetry.addData("state", state)
        telemetry.addData("linVelX", poseVelocity.linearVel.x)
        telemetry.addData("linVelY", poseVelocity.linearVel.y)
        telemetry.addData("angVel", poseVelocity.angVel)

        telemetry.addData("toplinVelX", topx)
        telemetry.addData("toplinVelY", topy)
        telemetry.addData("topangVel", topheading)

        telemetry.addData("armPos", arm.getCurrentPosition())
        telemetry.addData("armSpeed", arm.get())
        telemetry.addData("claw", claw.getGrabber())
        telemetry.addData("elevatorSpeed", elevator.get())
        telemetry.addData("elevatorPos", elevator.getCurrentPosition())
        telemetry.addData("elevatorSetpoint", elevator.setpoint)
        telemetry.addData("elevatorMode", elevator.mode)
        telemetry.addData("atSetPoint", elevator.atSetPoint())
        telemetry.update()
        lastTime = currentTime
        //FtcDashboard.getInstance().sendTelemetryPacket(packet)
        super.run()
    }
}