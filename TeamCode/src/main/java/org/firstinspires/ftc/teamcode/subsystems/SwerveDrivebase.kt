package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.RevIMU
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import java.text.DecimalFormat
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign


class SwerveDrivebase(hardwareMap: HardwareMap): Drivebase() {
    private var swerveLF: SwerveModule
    private var swerveRF: SwerveModule
    private var swerveLR: SwerveModule
    private var swerveRR: SwerveModule
    private var gyro: RevIMU
    private var kinematics = SwerveDriveKinematics(
        DrivebaseConstants.Measurements.LF_POS,
        DrivebaseConstants.Measurements.RF_POS,
        DrivebaseConstants.Measurements.LR_POS,
        DrivebaseConstants.Measurements.RR_POS
    )
    private val f: DecimalFormat = DecimalFormat("0.000")
    private var encoderLeft: Encoder
    private var encoderRight: Encoder
    private var odometry: DifferentialOdometry

    private var ws = Array(4) { 0.0 }
    private var wa = Array(4) { 0.0 }
    private var max: Double = 0.0
    private var R: Double = 0.0
    private var maintainHeading: Boolean = false

    private lateinit var hubs: List<LynxModule>

    init {
        val IDs = DrivebaseConstants.DeviceIDs

        hubs = hardwareMap.getAll(LynxModule::class.java)

        for (hub: LynxModule in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        gyro = RevIMU(hardwareMap)
        gyro.init()
        gyro.reset()

        swerveLF = SwerveModule(hardwareMap, IDs.LF_DRIVE_MOTOR, IDs.LF_TURN_MOTOR, IDs.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET, direction = DcMotorSimple.Direction.FORWARD)
        swerveRF = SwerveModule(hardwareMap, IDs.RF_DRIVE_MOTOR, IDs.RF_TURN_MOTOR, IDs.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        swerveLR = SwerveModule(hardwareMap, IDs.LR_DRIVE_MOTOR, IDs.LR_TURN_MOTOR, IDs.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET, direction = DcMotorSimple.Direction.FORWARD)
        swerveRR = SwerveModule(hardwareMap, IDs.RR_DRIVE_MOTOR, IDs.RR_TURN_MOTOR, IDs.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)

        encoderLeft = swerveLR.getDriveEncoder()
        encoderRight = swerveRF.getDriveEncoder()

        odometry = DifferentialOdometry(
            { encoderLeft.position.toDouble() },
            { encoderRight.position.toDouble() },
            DrivebaseConstants.Measurements.TRACK_WIDTH
        )

        R = hypot(DrivebaseConstants.Measurements.TRACK_WIDTH, DrivebaseConstants.Measurements.WHEEL_BASE)
    }

    /**
     * Translate drivebase
     */
    fun translate(angle: Double, speed: Double) {
        swerveLF.setAngle(angle)
        swerveRF.setAngle(angle)
        swerveLR.setAngle(angle)
        swerveRR.setAngle(angle)

        swerveLF.setSpeed(speed)
        swerveRF.setSpeed(speed)
        swerveLR.setSpeed(speed)
        swerveRR.setSpeed(speed)
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
        val heading = gyro.heading
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

    /**
     * This is brand new and completely untested, ignore
     */
    fun setKooky(pose: Pose2d) {
        val WHEEL_BASE = DrivebaseConstants.Measurements.WHEEL_BASE
        val TRACK_WIDTH = DrivebaseConstants.Measurements.TRACK_WIDTH

        val x = pose.x
        val y = pose.y
        val head = pose.heading

        val a: Double = x - head * (WHEEL_BASE / R)
        val b: Double = x + head * (WHEEL_BASE / R)
        val c: Double = y - head * (TRACK_WIDTH / R)
        val d: Double = y + head * (TRACK_WIDTH / R)

        ws = arrayOf(hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c))
        if (!maintainHeading) wa =
            arrayOf(atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c))

        max = ws.max()
    }

    /**
     * This is brand new and completely untested, ignore
     */
    /*
    fun writeKooky() {
        if (abs(max) > 1) ws[0] /= max
        swerveLF.setKookySpeed(abs(ws[0]))
        swerveLF.setKookyAngle(Rotation2d(wa[0].mod(2 * PI)))

        if (abs(max) > 1) ws[1] /= max
        swerveRF.setKookySpeed(abs(ws[1]))
        swerveRF.setKookyAngle(Rotation2d(wa[1].mod(2 * PI)))

        if (abs(max) > 1) ws[2] /= max
        swerveLR.setKookySpeed(abs(ws[2]))
        swerveLR.setKookyAngle(Rotation2d(wa[2].mod(2 * PI)))

        if (abs(max) > 1) ws[3] /= max
        swerveRR.setKookySpeed(abs(ws[3]))
        swerveRR.setKookyAngle(Rotation2d(wa[3].mod(2 * PI)))

    }

     */

    /**
     * This is brand new and completely untested, ignore
     */
    /*
    fun updateModulesKooky() {
        swerveLF.updateKooky()
        swerveRF.updateKooky()
        swerveLR.updateKooky()
        swerveRR.updateKooky()
    }

     */

    fun maintainHeading() {
        swerveLF.maintainHeading()
        swerveRF.maintainHeading()
        swerveLR.maintainHeading()
        swerveRR.maintainHeading()
        maintainHeading = true
    }

    fun turnInplace(speed: Double) {
        swerveLF.setAngle(45.0)
        swerveRF.setAngle(135.0)
        swerveLR.setAngle(-45.0)
        swerveRR.setAngle(-135.0)

        swerveLF.setSpeed(speed)
        swerveRF.setSpeed(speed)
        swerveLR.setSpeed(speed)
        swerveRR.setSpeed(speed)

    }

    fun translateTurn(angle: Double, translateSpeed: Double, turnSpeed: Double) {
        val turnAngle: Double = turnSpeed * 45.0

        if (closestAngle(angle, 135.0) >= 90.0) {
            swerveLF.setAngle(angle + turnAngle)
        } else {
            swerveLF.setAngle(angle - turnAngle)
        }

        if (closestAngle(angle, 225.0) >= 90.0) {
            swerveLR.setAngle(angle + turnAngle)
        } else {
            swerveLR.setAngle(angle - turnAngle)
        }

        if (closestAngle(angle, 45.0) > 90.0) {
            swerveRF.setAngle(angle + turnAngle)
        } else {
            swerveRF.setAngle(angle - turnAngle)
        }

        if (closestAngle(angle, 315.0) > 90.0) {
            swerveRR.setAngle(angle + turnAngle)
        } else {
            swerveRR.setAngle(angle - turnAngle)
        }

        swerveLF.setSpeed(translateSpeed)
        swerveRF.setSpeed(translateSpeed)
        swerveLR.setSpeed(translateSpeed)
        swerveRR.setSpeed(translateSpeed)

    }

    fun drive(angle: Double, translateSpeed: Double, turnSpeed: Double) {
        if ((translateSpeed == 0.0) && (turnSpeed != 0.0)) {
            turnInplace(turnSpeed)
        } else {
            translateTurn(angle, translateSpeed, turnSpeed)
        }
    }

    private fun closestAngle(a: Double, b: Double): Double {
        var dir: Double = b.mod(360.0) - a.mod(360.0)
        if (abs(dir) > 180.0) {
            dir = -(sign(dir) * 360.0) + dir
        }
        return dir
    }


    fun status(): String {
        val lf = "angle: " + f.format(swerveLF.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveLF.getSpeed()) + ", turnPower: " + f.format(swerveLF.getTurnPower())
        val rf = "angle: " + f.format(swerveRF.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveRF.getSpeed()) + ", turnPower: " + f.format(swerveRF.getTurnPower())
        val lr = "angle: " + f.format(swerveLR.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveLR.getSpeed()) + ", turnPower: " + f.format(swerveLR.getTurnPower())
        val rr = "angle: " + f.format(swerveRR.getAngle()) + ", desiredAngle: " + f.format(swerveLF.getDesiredAngle()) + ", speed: " + f.format(swerveRR.getSpeed()) + ", turnPower: " + f.format(swerveRR.getTurnPower())

        val status = lf + "\n" + rf + "\n" + lr + "\n" + rr

        return status
    }

    fun getHeading(): Double {
        return gyro.heading
    }

    fun resetGyro() {
        gyro.reset()
    }

    fun getPose(): Pose2d {
        return odometry.pose
    }

    fun setPose(pose2d: Pose2d) {
        odometry.updatePose(pose2d)
    }

    fun getAngularVelocity(): AngularVelocity {
        return gyro.revIMU.angularVelocity
    }

    fun periodic() {
        odometry.updatePose()
        swerveLF.update()
        swerveLR.update()
        swerveRF.update()
        swerveRR.update()
    }

}