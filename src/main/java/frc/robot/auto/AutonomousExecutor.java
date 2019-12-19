package frc.robot.auto;

import java.util.function.Consumer;

public class AutonomousExecutor implements Consumer<String> {

    public static final int SIGNAL_LENGTH = 3;

    public AutonomousExecutor() {

    }

    @Override
    public void accept(String s) {
        //process signal here
    }
}
