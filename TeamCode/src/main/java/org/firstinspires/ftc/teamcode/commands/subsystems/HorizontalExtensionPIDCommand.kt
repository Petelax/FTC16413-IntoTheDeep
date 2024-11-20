package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDFController
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants.*
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension

class HorizontalExtensionPIDCommand(private var elevator: HorizontalExtension, private val setPoint: Double): CommandBase() {
    private var controller: PIDFController
    init {
        val c = HorizontalConstants.HorizontalExtensionCoefficients
        controller = PIDFController(c.KP, c.KI, c.KD, c.KF)
        controller.setPoint = setPoint
        controller.setTolerance(HorizontalConstants.HorizontalExtensionConstants.POSITION_TOLERANCE, HorizontalConstants.HorizontalExtensionConstants.VELOCITY_TOLERANCE)
        addRequirements(elevator)
    }
    override fun initialize() {
        controller.calculate(elevator.getPosition(), setPoint)
    }

    override fun execute() {
        elevator.setSpeed(controller.calculate(elevator.getPosition(), setPoint))
    }

    override fun isFinished(): Boolean {
        return controller.atSetPoint()
    }

    override fun end(interrupted: Boolean) {
        elevator.setSpeed(0.0)
    }

}