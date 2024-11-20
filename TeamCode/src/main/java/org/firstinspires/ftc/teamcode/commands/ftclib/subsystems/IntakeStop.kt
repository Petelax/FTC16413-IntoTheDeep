package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake

class IntakeStop(private var intake: Intake): CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.setSpeed(0.0)
    }

    override fun execute() {
        intake.setSpeed(0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {

    }
}