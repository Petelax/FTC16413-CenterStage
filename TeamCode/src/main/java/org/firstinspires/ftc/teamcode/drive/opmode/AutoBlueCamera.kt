package org.firstinspires.ftc.teamcode.drive.opmode

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.commands.ArmPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.ClawCommand
import org.firstinspires.ftc.teamcode.drive.commands.DriveTime
import org.firstinspires.ftc.teamcode.drive.commands.DriveToPosition
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorNoVeloCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.drive.commands.WristCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.firstinspires.ftc.teamcode.vision.Side
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.PI

@Autonomous
class AutoBlueCamera: CommandOpMode() {
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
    private lateinit var propPipeline: PropPipeline
    private lateinit var portal: VisionPortal
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

        Globals.COLOR = Side.BLUE

        propPipeline = PropPipeline()
        portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .setCameraResolution(Size(1920, 1080))
            .setCamera(BuiltinCameraDirection.BACK)
            .addProcessor(propPipeline)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()

        FtcDashboard.getInstance().startCameraStream(propPipeline, 20.0)

        while (!isStarted) {
            telemetry.addData("location", propPipeline.location)
            telemetry.update()
        }

        val side = propPipeline.location
        portal.close()

        val spikeTranslation = Translation2d(-22.0, 0.0)
        var spikePose: Pose2d = when (side) {
            Side.LEFT -> {
                Pose2d(spikeTranslation, Rotation2d(PI/4))
            }
            Side.CENTER -> {
                Pose2d(spikeTranslation, Rotation2d(0.0))
            }
            Side.RIGHT -> {
                Pose2d(spikeTranslation, Rotation2d(-PI/4))
            }
            else -> {
                Pose2d(spikeTranslation, Rotation2d(0.0))
            }
        }

        schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ConditionalCommand(
                        DriveToPosition( drivebase, Pose2d(-32.0, 4.0, Rotation2d(0.0)), voltage::getVoltage ),
                        DriveToPosition( drivebase, Pose2d(-32.0, 0.0, Rotation2d(0.0)), voltage::getVoltage ),
                    ) { side == Side.RIGHT },
                    SequentialCommandGroup(
                        ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED).withTimeout(5000),
                        WaitCommand(500),
                        ElevatorPIDCommand(elevator, Elevator.ROTATE),
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.TOP),
                    )
                ),
                DriveToPosition(drivebase, spikePose, voltage::getVoltage),
                ElevatorPIDCommand(elevator, Elevator.BOTTOM),

                ConditionalCommand(
                    InstantCommand({claw.setLeft(DrivebaseConstants.GrabberPositions.OPEN)}),
                    InstantCommand({claw.setLeft(DrivebaseConstants.GrabberPositions.CLOSED)}),
                ) { side != Side.RIGHT },


                WaitCommand(100),
                DriveToPosition(drivebase, Pose2d(spikePose.translation, Rotation2d(-PI/2)), voltage::getVoltage),
                ParallelCommandGroup(
                    DriveToPosition(drivebase, Pose2d(-32.0, 34.0, Rotation2d(-PI/2)), voltage::getVoltage),
                ),
                ParallelCommandGroup(
                    WristCommand(wrist, DrivebaseConstants.WristPositions.TOP),
                    ElevatorPIDCommand(elevator, Elevator.MIDDLE_PLACE),
                ),
                ConditionalCommand(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED),
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN),
                ) { side == Side.RIGHT },

                WaitCommand(200),

                ParallelCommandGroup(
                    ElevatorNoVeloCommand(elevator, Elevator.ROTATE),
                    ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                    WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
                ),
                ElevatorPIDCommand(elevator, Elevator.BOTTOM),

                DriveToPosition(drivebase, Pose2d(-20.0, 34.0, Rotation2d(-PI/2)), voltage::getVoltage),


            )
        )

        /*
        schedule(
            SequentialCommandGroup(

                DriveToPosition(drivebase, Pose2d(-25.0, -34.5, Rotation2d(PI/2)), voltage::getVoltage),

                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN).withTimeout(2000),
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM+0.01),
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.CLOSED).withTimeout(5000),
                    WaitCommand(500),
                    ElevatorPIDCommand(elevator, Elevator.ROTATE),
                    //ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.MIDDLE),
                    ParallelCommandGroup(
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.TOP),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.TOP)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.TOP * 2/3),
                ),


                SequentialCommandGroup(
                    ClawCommand(claw, DrivebaseConstants.GrabberPositions.OPEN),
                    WaitCommand(150),
                    //DriveToPosition(drivebase, Pose2d(-25.0, -25.0, Rotation2d(PI/2)), voltage::getVoltage),
                    DriveTime(drivebase, ChassisSpeeds(0.7, 0.0, 0.0), 0.5),
                    ParallelCommandGroup(
                        ElevatorNoVeloCommand(elevator, Elevator.ROTATE),
                        ArmPIDCommand(arm, DrivebaseConstants.ArmPositions.BOTTOM),
                        WristCommand(wrist, DrivebaseConstants.WristPositions.BOTTOM)
                    ),
                    ElevatorPIDCommand(elevator, Elevator.BOTTOM),
                ),

                DriveToPosition(drivebase, Pose2d(0.0, -34.0, Rotation2d(PI/2)), voltage::getVoltage),

            )
        )

         */

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