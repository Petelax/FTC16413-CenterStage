package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.wpilib.HolonomicDriveController
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile
import org.firstinspires.ftc.teamcode.wpilib.WpilibPIDController

class HolonomicDrive(drivebase: SwerveDrivebase): CommandBase() {
    private var drivebase: SwerveDrivebase
    private var controller: HolonomicDriveController
    private var xController: WpilibPIDController
    private var yController: WpilibPIDController
    private var headingController: ProfiledPIDController

    init {
        this.drivebase = drivebase
        val c = DrivebaseConstants.DrivebasePIDConstants
        val h = DrivebaseConstants.DrivebasePIDConstants.Heading
        xController = WpilibPIDController(c.kP, c.kI, c.kD)
        yController = WpilibPIDController(c.kP, c.kI, c.kD)
        headingController = ProfiledPIDController(h.kP, h.kI, h.kD, TrapezoidProfile.Constraints(h.MAX_VELO, h.MAX_ACCEL))
        controller = HolonomicDriveController(
            xController,
            yController,
            headingController
        )

    }

    override fun initialize() {

    }

    override fun execute() {
        //controller.calculate(drivebase.getPose(), )
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }
}