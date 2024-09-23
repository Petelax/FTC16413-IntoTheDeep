package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Pose2d
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

class PIDToPosition(private val drive: SwerveDrivetrain, private val setpoint: Pose2d): CommandBase() {
    val xController: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    val yController: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)
    val headingController: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        xController.setPoint = setpoint.x
        yController.setPoint = setpoint.y
        headingController.setPoint = setpoint.heading

    }

    override fun execute() {

    }

    override fun isFinished(): Boolean {
        return true
    }

    override fun end(interrupted: Boolean) {

    }

}