package newLineFollower;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Date;


public class LineFollower {

    LineFollower() {
        try {
            String filename = "/home/lejos/programs/output-" + new Date().getTime() + ".txt";
            File file = new File(filename);

            if (!file.exists()) {
                file.createNewFile();
            }

            file.setWritable(true);

            this.fstream = new FileWriter(filename);
            this.out = new BufferedWriter(this.fstream);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    protected static EV3ColorSensor leftSensor;
    protected static EV3ColorSensor rightSensor;
    protected static int colorSampleSize;

    FileWriter fstream = null;
    BufferedWriter out = null;

    protected void initializeEngines() {
        // 1: left, 2: right
        leftSensor = new EV3ColorSensor(SensorPort.S1);
        rightSensor = new EV3ColorSensor(SensorPort.S2);
        // SampleSize is the same for every sensor, because we use the same sensors in all three locations
        colorSampleSize = leftSensor.sampleSize();
        Motor.A.setSpeed(50);
        Motor.D.setSpeed(50);
    }

    protected static int getState() {
        float[] leftSample = new float[colorSampleSize];
        float[] rightSample = new float[colorSampleSize];

        // Fetch Color values from sensor
        leftSensor.fetchSample(leftSample, 0);
        rightSensor.fetchSample(rightSample, 0);

        int colorLeftSensorDetected;
        int colorRightSensorDetected;

        int leftColour = (int) leftSample[0];
        int rightColour = (int) rightSample[0];

        // Only check for black values (value for black is a 7) everything else we will consider as the same values
        if (leftColour == 7 || leftColour == -1) {
            colorLeftSensorDetected = 2;
        } else {
            colorLeftSensorDetected = 0;
        }

        if (rightColour == 7 || rightColour == -1) {
            colorRightSensorDetected = 1;
        } else {
            colorRightSensorDetected = 0;
        }

        int colourResult = colorLeftSensorDetected + colorRightSensorDetected;
        return colourResult;
    }

    protected final static int LEFT_CURVE = 0;
    protected final static int LINE_FOLLOW = 1;
    protected final static int RIGHT_CURVE = 2;

    protected static void execute(int action) {
        switch(action) {
            case LEFT_CURVE:
                turnLeft();
                break;
            case LINE_FOLLOW:
                forward();
                break;
            case RIGHT_CURVE:
                turnRight();
                break;
            default:
                break;
        }
    }

    // A is the left wheel and D is the right wheel
    protected static void forward() {
        Motor.A.forward();
        Motor.D.forward();
        Motor.A.setSpeed(90);
        Motor.D.setSpeed(90);
    }

    protected static void turnLeft() {
        Motor.A.backward();
        Motor.D.forward();
        Motor.A.setSpeed(45);
        Motor.D.setSpeed(45);
    }

    protected static void turnRight() {
        Motor.A.forward();
        Motor.D.backward();
        Motor.A.setSpeed(45);
        Motor.D.setSpeed(45);
    }

    protected static double getReward(int state) {
        double reward;
        switch(state) {
            case 0:
                reward = 0;
                return reward;
            case 1:
                reward = 0;
                return reward;
            case 2:
                reward = 0;
                return reward;
            case 3:
                reward = 1;
                return reward;
            default:
                return 0;
        }
    }

    // Initialize values required for Q-Learning
    private final static double alpha = 0.9; // learning rate, alpha 0 1.0 means completely rewrite old value
    private final static double gamma = 0.2; // discount value, short term success vs long term success
    private final static double epsilon = 0.01; // exploration rate, where robot will randomly move despite a potentially better option

    // We have 7 possible states and for each state there are 3 possible actions (in this particular case)
    // first value: states, second value: actions
    // e.g. [2][1] would equal: state = 2; action = 1 = forward
    static double [][] QValues = {{0, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}};

    protected static void initQValues() {
        for(int i = 0; i < QValues.length; i++) {
            for(int j = 0; j < QValues[0].length; j++) {
                QValues[i][j] = Math.random();
            }
        }
    }

    protected void qlearning() {
        initQValues();
        int newState = -1;
        int roundCounter = 0;
        // We do not have an end state yet so we just let the robot drive all day long
        while(true) {
            roundCounter++;
            int state = (newState > -1) ? newState : getState();

            double maxUtil = -1000; //just some Value to begin the process
            int action = 1;

            // We cycle through the array to find the action with the highest reward for the given state
            for(int i=0; i < 3; i++) {
                if(QValues[state][i] > maxUtil) {
                    maxUtil = QValues[state][i];
                    action = i;
                }
            }

            // no action has been found above, that is better than the others or if the randValue is below the
            // exploration value, a random number between 0 and 2 will be created for the action

            if(Math.random() <= epsilon) {
                action = (int) (Math.random()*3);
            }

            execute(action);

            try {
                Thread.sleep(180);
            } catch(Exception e) {
                e.printStackTrace();
            }

            int nextState = getState();

            double reward = getReward(nextState);

            maxUtil = -1000; // Reset the value to go for the second run

            // Same as above, however we use the nextState instead of the start state
            for(int i=0; i < 3; i++) {
                if(QValues[nextState][i] > maxUtil) {
                    maxUtil = QValues[nextState][i];
                }
            }

            // Updating value of matrix for the initial state and action pair based on reward, previous value and potential
            // util values for the current state
            // Typical formula for the calculation of the QValue for a given state and action
            QValues[state][action] =
                    (QValues[state][action] + alpha * (reward + (gamma * maxUtil) - QValues[state][action]));

            LCD.drawString(actionToDirection(getMaxActionFromState(0)) + "," +actionToDirection(getMaxActionFromState(1)) + "," + actionToDirection(getMaxActionFromState(2)) + "," + actionToDirection(getMaxActionFromState(3)), 0, 0);
            LCD.drawString("00: " + actionToDirection(getMaxActionFromState(0)) + " " + QValues[0][getMaxActionFromState(0)], 0, 1);
            LCD.drawString("01: " + actionToDirection(getMaxActionFromState(1)) + " " + QValues[1][getMaxActionFromState(1)], 0, 2);
            LCD.drawString("10: " + actionToDirection(getMaxActionFromState(2)) + " " + QValues[2][getMaxActionFromState(2)], 0, 3);
            LCD.drawString("11: " + actionToDirection(getMaxActionFromState(3)) + " " + QValues[3][getMaxActionFromState(3)], 0, 4);
            LCD.drawString("State: " + nextState, 0,5);

            try {
                if ((roundCounter % 50) == 0) {
//                    out.write("" + roundCounter + ","
//                            + actionToDirection(getMaxActionFromState(0)) + ","
//                            + actionToDirection(getMaxActionFromState(1)) + ","
//                            + actionToDirection(getMaxActionFromState(2)) + ","
//                            + actionToDirection(getMaxActionFromState(3)) + "\\n");
                    out.write(worldToJSON(roundCounter) + ",");
                    out.flush();
                }
            } catch (Exception e) {
                e.printStackTrace();
            }

            try {
                Thread.sleep(180);
            } catch(Exception e) {
                e.printStackTrace();
            }
        }
    }

    private String actionToDirection(int action) {
        if (action == 0) {
            return "L";
        } else if (action == 1) {
            return "G";
        } else {
            return "R";
        }
    }

    private int getMaxActionFromState(int state) {
        double max = QValues[state][0];
        int idx = 0;
        for (int i = 1; i < QValues[state].length; i++) {
            if (max < QValues[state][i]) {
                max = QValues[state][i];
                idx = i;
            }
        }
        return idx;
    }

    private String worldToJSON(int step) {
        String tmp = "{";

        tmp += "step:" + step + ",";
        tmp += "values:" + qValuesToJSON();

        return tmp + "}";
    }

    private String qValuesToJSON() {
        String tmp = "[";

        for (int i = 0; i < QValues.length; i++) {
            String valTmp = "{";

            valTmp += "left:" + QValues[i][0] + ",";
            valTmp += "forward:" + QValues[i][1] + ",";
            valTmp += "rigth:" + QValues[i][2];

            tmp += valTmp + "}" + (i != QValues.length - 1 ? "," : "");
        }

        return tmp + "]";
    }

    public static void main(String[] args) {
        LineFollower lf = new LineFollower();
        lf.initializeEngines();
        lf.qlearning();
    }

}