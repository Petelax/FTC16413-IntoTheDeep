package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDFController
import org.firstinspires.ftc.teamcode.constants.VerticalConstants.*
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator

class ElevatorPIDCommand(private var elevator: Elevator, private val setPoint: Double): CommandBase() {
    private var controller: PIDFController
    init {
        val c = ElevatorCoefficients
        controller = PIDFController(c.KP, c.KI, c.KD, c.KF)
        controller.setPoint = setPoint
        controller.setTolerance(ElevatorConstants.POSITION_TOLERANCE, ElevatorConstants.VELOCITY_TOLERANCE)
        addRequirements(elevator)
    }
    override fun initialize() {
        controller.calculate(elevator.getPosition(), setPoint)
    }

    override fun execute() {
        val goingDown = if (setPoint <= ElevatorPositions.LOWER_LIMIT) { -ElevatorCoefficients.KG } else { 0.0 }
        elevator.setSpeed(controller.calculate(elevator.getPosition(), setPoint) + goingDown)
    }

    override fun isFinished(): Boolean {
        return if(setPoint <= ElevatorPositions.LOWER_LIMIT) { elevator.atBottom() } else {controller.atSetPoint()}
    }

    override fun end(interrupted: Boolean) {
        elevator.setSpeed(0.0)
    }

}