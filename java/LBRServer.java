package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.fri.FRIChannelInformation;
import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRIJointOverlay;
import com.kuka.fri.FRISession;
import com.kuka.fri.IFRISessionListener;
import com.kuka.fri.common.ClientCommandMode;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.sensitivity.LBR;
import com.kuka.sensitivity.controlmode.JointImpedanceControlMode;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import javax.inject.Inject;

public class LBRServer extends RoboticsAPIApplication {
  private String clientName;
  private FRISession friSession;
  private FRIConfiguration friConfig;
  private FRIJointOverlay motionOverlay;
  private AbstractMotionControlMode controlMode;
  private Boolean initSuccess; // true when initialization was successful

  @Inject private IApplicationUI appUi;
  @Inject private LBR robot;

  @Override
  public void initialize() {

    // Set class attributes
    // clientName = ""; // YOUR IP HERE

    // Specify method variables
    int usrin;

    /*
     * Configure the FRI
     */

    // Setup configuration object
    friConfig = FRIConfiguration.createRemoteConfiguration(robot, clientName);

    // Sampling frequency
    usrin =
        appUi.displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Sampling frequency (Hz)?",
            "100",
            "200",
            "500",
            "1000");

    switch (usrin) {
      case 0:
        break; // default is 10ms
      case 1:
        friConfig.setSendPeriodMilliSec(5);
        break;
      case 2:
        friConfig.setSendPeriodMilliSec(2);
        break;
      case 3:
        friConfig.setSendPeriodMilliSec(1);
        break;
      default:
        break;
    }

    // Report
    getLogger().info("FRI Configuration");
    getLogger().info("    Host Name: " + friConfig.getHostName());
    getLogger().info("    Port on controller: " + String.valueOf(friConfig.getPortOnController()));
    getLogger().info("    Port on remote: " + String.valueOf(friConfig.getPortOnRemote()));
    getLogger().info("    Recieve multiplier: " + String.valueOf(friConfig.getReceiveMultiplier()));
    getLogger()
        .info("    Send period millisec: " + String.valueOf(friConfig.getSendPeriodMilliSec()));

    /*
     * Setup FRI session
     */

    friSession = new FRISession(friConfig);

    // Setup listener for changes to FRI session
    IFRISessionListener listener =
        new IFRISessionListener() {

          @Override
          public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
            getLogger().info("FRI connection quality changed");
            getLogger().info("    Quality: " + friChannelInformation.getQuality().name());
            getLogger().info("    Jitter:" + String.valueOf(friChannelInformation.getJitter()));
            getLogger().info("    Latency:" + String.valueOf(friChannelInformation.getLatency()));
          }

          @Override
          public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
            getLogger().info(" FRI session state changed");
            getLogger().info("    Quality: " + friChannelInformation.getQuality().name());
            getLogger().info("    Jitter:" + String.valueOf(friChannelInformation.getJitter()));
            getLogger().info("    Latency:" + String.valueOf(friChannelInformation.getLatency()));
          }
        };

    friSession.addFRISessionListener(listener);

    getLogger().info("FRI Session");
    getLogger().info("    ID: " + friSession.getFRISessionId());

    /*
     * Setup joint overlay
     */

    // Command mode
    usrin =
        appUi.displayModalDialog(
            ApplicationDialogType.QUESTION, "Client command mode?", "POSITION", "WRENCH", "TORQUE");

    int ansRate = friConfig.getReceiveMultiplier() * friConfig.getSendPeriodMilliSec();

    getLogger().info("Client command mode");
    switch (usrin) {
      case 0:
        getLogger().info("    Command mode: POSITION");
        motionOverlay = new FRIJointOverlay(friSession, ClientCommandMode.JOINT_POSITION);
        break;
      case 1:
        getLogger().warn("Client command mode WRENCH not yet implemented!");
        initSuccess = false;
        return;
        //            motionOverlay = new FRIJointOverlay(friSession, ClientCommandMode.WRENCH);
        //            if (ansRate > 5) {
        //
        //              getLogger().warn("Answer rate is " + String.valueOf(ansRate)+ "ms, WRENCH
        // command mode requires <= 5ms. Choose a different sampling frequency.");
        //              return;
        //            }
        //            break;
      case 2:
        getLogger().info("    Command mode: TORQUE");
        motionOverlay = new FRIJointOverlay(friSession, ClientCommandMode.TORQUE);
        if (ansRate > 5) {
          initSuccess = false;
          getLogger()
              .warn(
                  "Answer rate is "
                      + String.valueOf(ansRate)
                      + "ms, TORQUE command mode requires <= 5ms. Choose a different sampling"
                      + " frequency.");
          return;
        }
        break;
      default:
        break;
    }

    /*
     * Setup controller
     */

    usrin =
        appUi.displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Control mode?",
            "Position",
            "Cartesian Impedance",
            "Cartesian Sine Impedance",
            "Joint Impedance");

    getLogger().info("Controller");
    switch (usrin) {
      case 0:
        controlMode = new PositionControlMode();
        getLogger().info("    Control mode: position");
        break;
      case 1:
      case 2:
        initSuccess = false;
        getLogger().warn("Cartesian impedance controller not yet implemented!");
        return;
      case 3:
        double jointStiffness = 1000.0;
        double jointDamping = 0.7;

        // Setup joint stiffness
        usrin =
            appUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Stiffness gain?",
                "1000",
                "500",
                "200",
                "100",
                "50");

        switch (usrin) {
          case 0:
            jointStiffness = 1000.0;
            break;
          case 1:
            jointStiffness = 500.0;
            break;
          case 2:
            jointStiffness = 200.0;
            break;
          case 3:
            jointStiffness = 100.0;
            break;
          case 4:
            jointStiffness = 50.0;
            break;
          default:
            break;
        }

        // Setup joint damping
        usrin =
            appUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Damping gain?",
                "0.1",
                "0.25",
                "0.5",
                "0.75",
                "1.0");

        switch (usrin) {
          case 0:
            jointDamping = 0.1;
            break;
          case 1:
            jointDamping = 0.25;
            break;
          case 2:
            jointDamping = 0.5;
            break;
          case 3:
            jointDamping = 0.75;
            break;
          case 4:
            jointDamping = 1.0;
            break;
          default:
            break;
        }

        // Setup control mode
        controlMode =
            new JointImpedanceControlMode(
                jointStiffness,
                jointStiffness,
                jointStiffness,
                jointStiffness,
                jointStiffness,
                jointStiffness,
                jointStiffness);
        ((JointImpedanceControlMode) controlMode).setDampingForAllJoints(jointDamping);
        getLogger().info("    Control mode: joint impedance");
        double[] stiffness = ((JointImpedanceControlMode) controlMode).getStiffness();
        double[] damping = ((JointImpedanceControlMode) controlMode).getDamping();
        for (int j = 0; j < 7; j++) {
          getLogger()
              .info(
                  "    Joint axis "
                      + String.valueOf(j)
                      + " stiffness: "
                      + String.valueOf(stiffness[j]));
        }
        for (int j = 0; j < 7; j++) {
          getLogger()
              .info(
                  "    Joint axis "
                      + String.valueOf(j)
                      + " damping: "
                      + String.valueOf(damping[j]));
        }

      default:
        break;
    }

    // Connect to FRI
    long await_seconds = 10;
    getLogger()
        .info("Connecting to FRI (timeout is " + String.valueOf(await_seconds) + " seconds) ...");
    try {
      friSession.await(await_seconds, TimeUnit.SECONDS);
    } catch (final TimeoutException e) {
      initSuccess = false;
      getLogger().error(e.getLocalizedMessage());
      return;
    }

    getLogger().info("FRI connection established.");

    /*
     * Report completion of initialization
     */

    initSuccess = true;
    getLogger().info("LBRServer initialization complete.");
  }

  @Override
  public void dispose() {

    if (null != friSession) {
      friSession.close();
      getLogger().info("Closed connection to client.");
    }

    getLogger().info("Finished LBRServer.");
  }

  @Override
  public void run() {

    // Ensure initialization was successful
    if (!initSuccess) {
      getLogger().error("LBRServer initialization was not successful!");
      return;
    }

    // Start session
    long endlessTimeout = -1;
    PositionHold positionHold = new PositionHold(controlMode, endlessTimeout, null);
    robot.getFlange().move(positionHold.addMotionOverlay(motionOverlay));
  }
}
