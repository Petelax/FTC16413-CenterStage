package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.drive.commands.DriveToPosition
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import kotlin.math.PI

@Config
@TeleOp(group = "test")
class LocalizerTest: CommandOpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var intake: Intake
    private lateinit var elevator: Elevator
    private lateinit var wrist: Wrist
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var driverOp: GamepadEx
    private lateinit var toolOp: GamepadEx
    private var lastTime: Long = 0
    private lateinit var dashboard: FtcDashboard
    private lateinit var voltage: VoltageSensor
    @JvmField var startX = 12.0
    @JvmField var startY = 48+(24-7.0)
    @JvmField var startHeading = -PI/2
    //@JvmField var startPose = Pose2d(startX, startY, Rotation2d(startHeading))
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
        driverOp = GamepadEx(gamepad1)

        drivebase.defaultCommand = FieldCentricDrive(
            drivebase,
            {driverOp.leftY},
            {driverOp.leftX*-1.0},
            {driverOp.rightX*-1.0},
            {driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)},
            {true}
        )


        //drivebase.setPose(startPose)

    }

    override fun run() {
        val currentTime = System.nanoTime()

        val pose = drivebase.getPose()

        val packet = TelemetryPacket()
        packet.fieldOverlay().setFill("blue").fillRect(pose.x-7, pose.y-7, 14.0, 14.0).setRotation(pose.heading)
        packet.put("hz", 1E9/(currentTime-lastTime))
        packet.put("voltage", voltage.voltage)
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
