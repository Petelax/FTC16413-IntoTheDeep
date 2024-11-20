package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.controller.PIDFController
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants.ElevatorCoefficients
import org.firstinspires.ftc.teamcode.constants.VerticalConstants.ElevatorConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants.ElevatorPositions
import org.firstinspires.ftc.teamcode.subsystems.Elevator

class ElevatorPID(private val setPoint: Double) : Command {
    override val requirements: Set<Any> = setOf(Elevator)
    override val runStates: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.ACTIVE)

    private var controller: PIDFController

    init {
        val c = ElevatorCoefficients
        controller = PIDFController(c.KP, c.KI, c.KD, c.KF)
        controller.setPoint = setPoint
        controller.setTolerance(ElevatorConstants.POSITION_TOLERANCE, ElevatorConstants.VELOCITY_TOLERANCE)
    }

    override fun initialise() {
        controller.calculate(Elevator.getPosition(), setPoint)
    }

    override fun execute() {
        val goingDown = if (setPoint <= ElevatorPositions.LOWER_LIMIT) { -ElevatorCoefficients.KG } else { 0.0 }
        Elevator.setSpeed(controller.calculate(Elevator.getPosition(), setPoint) + goingDown)
    }

    override fun finished(): Boolean {
        return if(setPoint <= ElevatorPositions.LOWER_LIMIT) { Elevator.atBottom() } else {controller.atSetPoint()}
    }

    override fun end(interrupted: Boolean) {
        Elevator.setSpeed(0.0)
    }

    override fun toString(): String {
        return "elevator-pid"
    }
}