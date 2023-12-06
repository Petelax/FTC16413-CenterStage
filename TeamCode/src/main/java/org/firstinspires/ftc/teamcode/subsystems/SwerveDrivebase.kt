package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.RevIMU
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.Units
import org.firstinspires.ftc.teamcode.rr.TwoDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import java.text.DecimalFormat
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign


class SwerveDrivebase(hardwareMap: HardwareMap): SubsystemBase() {
    private var swerveLF: SwerveModule
    private var swerveRF: SwerveModule
    private var swerveLR: SwerveModule
    private var swerveRR: SwerveModule
    //private var gyro: RevIMU
    private var kinematics = SwerveDriveKinematics(
        DrivebaseConstants.Measurements.LF_POS,
        DrivebaseConstants.Measurements.RF_POS,
        DrivebaseConstants.Measurements.LR_POS,
        DrivebaseConstants.Measurements.RR_POS
    )
    private val f: DecimalFormat = DecimalFormat("0.000")
    private var encoderLeft: Encoder
    private var encoderRight: Encoder
    private var localizer: TwoDeadWheelLocalizer
    private var pose: com.acmerobotics.roadrunner.Pose2d = com.acmerobotics.roadrunner.Pose2d(0.0, 0.0, 0.0)
    private var poseVelocity: PoseVelocity2d = PoseVelocity2d(com.acmerobotics.roadrunner.Vector2d(0.0, 0.0), 0.0)
    private var poseAccel: PoseVelocity2d = PoseVelocity2d(com.acmerobotics.roadrunner.Vector2d(0.0, 0.0), 0.0)
    private var lastVel = poseVelocity
    private var lastTime = System.nanoTime()

    private var maintainHeading: Boolean = false

    private var hubs: List<LynxModule>

    private val INCHES_PER_TICK = (1.377953 * PI) / 8192
    private var imu: IMU
    private var imuOffset: Double = 0.0

    init {
        val IDs = DrivebaseConstants.DeviceIDs

        hubs = hardwareMap.getAll(LynxModule::class.java)

        for (hub: LynxModule in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        /*
        gyro = RevIMU(hardwareMap)
        gyro.init()
        gyro.reset()
         */

        swerveLF = SwerveModule(hardwareMap, IDs.LF_DRIVE_MOTOR, IDs.LF_TURN_MOTOR, IDs.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET, direction = DcMotorSimple.Direction.FORWARD)
        swerveRF = SwerveModule(hardwareMap, IDs.RF_DRIVE_MOTOR, IDs.RF_TURN_MOTOR, IDs.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        swerveLR = SwerveModule(hardwareMap, IDs.LR_DRIVE_MOTOR, IDs.LR_TURN_MOTOR, IDs.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET, direction = DcMotorSimple.Direction.FORWARD)
        swerveRR = SwerveModule(hardwareMap, IDs.RR_DRIVE_MOTOR, IDs.RR_TURN_MOTOR, IDs.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)

        /* THIS IS DANGER */
        swerveRF.setPID(0.009, DrivebaseConstants.TurnPIDConstants.kI, DrivebaseConstants.TurnPIDConstants.kD)

        encoderLeft = swerveLR.getDriveEncoder()
        encoderRight = swerveRR.getDriveEncoder()

        imu = hardwareMap.get<IMU>(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )
        imu.initialize(parameters)

        imu.resetYaw()

        localizer = TwoDeadWheelLocalizer(hardwareMap, imu, INCHES_PER_TICK)

    }

    /**
     * The actual main drive method I've been using
     */
    fun kinematicsDrive(speeds: ChassisSpeeds) {
        val moduleStates: Array<SwerveModuleState> = kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        swerveLF.setSwerveModuleState(moduleStates[0])
        swerveRF.setSwerveModuleState(moduleStates[1])
        swerveLR.setSwerveModuleState(moduleStates[2])
        swerveRR.setSwerveModuleState(moduleStates[3])
    }

    fun fieldCentricDrive(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double) {
        val heading = getHeading()
        val vx = MathUtil.clamp(strafeSpeed,-1.0, 1.0)
        val vy = MathUtil.clamp(forwardSpeed,-1.0, 1.0)
        var omega = MathUtil.clamp(turnSpeed,-1.0, 1.0)

        if (vx < 0.001 && vy < 0.001) {
            omega *= 2
        }

        val input = Vector2d(vx, vy).rotateBy(-heading)

        val speeds = ChassisSpeeds(
            input.x*DrivebaseConstants.Measurements.MAX_VELOCITY,
            input.y*DrivebaseConstants.Measurements.MAX_VELOCITY,
            omega*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY)

        val moduleStates: Array<SwerveModuleState> = kinematics.toSwerveModuleStates(speeds)
        //SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        swerveLF.setSwerveModuleState(moduleStates[0])
        swerveRF.setSwerveModuleState(moduleStates[1])
        swerveLR.setSwerveModuleState(moduleStates[2])
        swerveRR.setSwerveModuleState(moduleStates[3])
    }

    fun maintainHeading() {
        swerveLF.maintainHeading()
        swerveRF.maintainHeading()
        swerveLR.maintainHeading()
        swerveRR.maintainHeading()
        maintainHeading = true
    }

    fun lock() {
        swerveLF.setAngle(-45.0)
        swerveRF.setAngle(45.0)
        swerveLR.setAngle(-45.0)
        swerveRR.setAngle(45.0)

        swerveLF.setSpeed(0.0)
        swerveRF.setSpeed(0.0)
        swerveLR.setSpeed(0.0)
        swerveRR.setSpeed(0.0)
    }

    fun status(): String {
        val lf = "angle: " + f.format(swerveLF.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveLF.getSpeed()) + ", turnPower: " + f.format(swerveLF.getTurnPower())
        val rf = "angle: " + f.format(swerveRF.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveRF.getSpeed()) + ", turnPower: " + f.format(swerveRF.getTurnPower())
        val lr = "angle: " + f.format(swerveLR.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveLR.getSpeed()) + ", turnPower: " + f.format(swerveLR.getTurnPower())
        val rr = "angle: " + f.format(swerveRR.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveRR.getSpeed()) + ", turnPower: " + f.format(swerveRR.getTurnPower())

        val status = lf + "\n" + rf + "\n" + lr + "\n" + rr

        return status
    }

    /**
     * @return degrees
     */
    fun getHeading(): Double {
        return ((imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES) + imuOffset) * 1.0) % (360)
    }

    fun getHeadingRad(): Double {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }

    fun resetGyro() {
        imu.resetYaw()
    }

    fun getPose(): Pose2d {
        return Pose2d(pose.position.x, pose.position.y, Rotation2d(Units.degreesToRadians(getHeading())))
    }

    fun getPoseVelocity(): PoseVelocity2d {
        return poseVelocity
    }

    fun getPoseAccel(): PoseVelocity2d {
        val currentTime = System.nanoTime()
        poseVelocity = updatePoseEstimate()
        val x = (poseVelocity.linearVel.x-lastVel.linearVel.x) / (currentTime-lastTime)
        val y = (poseVelocity.linearVel.y-lastVel.linearVel.y) / (currentTime-lastTime)
        val w = (poseVelocity.linearVel.y-lastVel.angVel) / (currentTime-lastTime)
        poseAccel = PoseVelocity2d(com.acmerobotics.roadrunner.Vector2d(x, y), w)
        lastTime = currentTime
        return poseAccel
    }

    fun setPose(pose2d: Pose2d) {
        pose = com.acmerobotics.roadrunner.Pose2d(pose2d.x, pose2d.y, pose2d.rotation.degrees)
        imuOffset = pose2d.rotation.degrees
    }

    fun getAngularVelocity(): AngularVelocity {
        //return gyro.revIMU.angularVelocity
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES)
    }

    override fun periodic() {
        poseVelocity = updatePoseEstimate()
        swerveLF.update()
        swerveLR.update()
        swerveRF.update()
        swerveRR.update()
        lastVel = poseVelocity
    }

    private fun updatePoseEstimate(): PoseVelocity2d {
        val twist: Twist2dDual<Time> = localizer.update()
        pose = pose.plus(twist.value())

        return twist.velocity().value()
    }

}