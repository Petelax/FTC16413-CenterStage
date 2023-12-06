package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.commands.ArmPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.ClawCommand
import org.firstinspires.ftc.teamcode.drive.commands.DriveToPosition
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorNoVeloCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.PathFollowing
import org.firstinspires.ftc.teamcode.drive.commands.TurnInPlace
import org.firstinspires.ftc.teamcode.drive.commands.WristCommand
import org.firstinspires.ftc.teamcode.drive.test.CommandBasedNotBasedTest
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import kotlin.math.PI

@Autonomous
class TrajAuto: CommandOpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var intake: Intake
    private lateinit var elevator: Elevator
    private lateinit var wrist: Wrist
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private var lastTime: Long = 0
    private lateinit var dashboard: FtcDashboard
    private lateinit var voltage: VoltageSensor
    //private val startPose = Pose2d(-65.0, 12.0, Rotation2d(-PI/2))

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        dashboard = FtcDashboard.getInstance()
        drivebase = SwerveDrivebase(hardwareMap)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        wrist = Wrist(hardwareMap)
        claw = Claw(hardwareMap)
        arm = Arm(hardwareMap)
        intake = Intake(hardwareMap)
        voltage = hardwareMap.get(VoltageSensor::class.java, "Control Hub")

        //drivebase.setPose(startPose)
        //drivebase.setPose(Pose2d(0.0, 0.0, Rotation2d(-PI/2)))

        val pose = drivebase.getPose()
        val packet = TelemetryPacket()
        packet.fieldOverlay().setFill("blue").setRotation(pose.heading).fillRect(pose.x-7.0, pose.y-7.0, 14.0, 14.0)
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        dashboard.sendTelemetryPacket(packet)

        schedule(
            SequentialCommandGroup(
                //TurnInPlace(drivebase, Rotation2d(PI/2), voltage::getVoltage),
                PathFollowing(drivebase, Pose2d(12.0, 0.0, Rotation2d(0.0)))
            )
        )

    }

    override fun run() {
        val currentTime = System.nanoTime()

        val pose = drivebase.getPose()

        val packet = TelemetryPacket()

        packet.fieldOverlay().setFill("blue").setRotation(pose.heading).fillRect(pose.x-7.0, pose.y-7.0, 14.0, 14.0)
        packet.put("hz", 1E9/(currentTime-lastTime))
        packet.put("voltage", voltage.voltage)
        packet.put("speed", drivebase.getAngularVelocity())
        packet.put("degrees", drivebase.getHeading())
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        dashboard.sendTelemetryPacket(packet)
        //telemetry.addData("hz ", 1E9/(currentTime-lastTime))
        //telemetry.update()
        lastTime = currentTime
        super.run()
    }
}
