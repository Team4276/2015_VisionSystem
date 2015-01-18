package display_cv;

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.event.EventHandler;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.scene.Group;
import javafx.scene.control.*;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

public class DisplayCv extends Application
{

    final static int CANVAS_WIDTH = 600;
    final static int CANVAS_HEIGHT = 600;
    final static int GAP_SIZE_PIXELS = 100;
    final static double FUNNEL_ANGLE_DEGREES = 15.0;
    final static double MAX_RANGE_FEET = 23.0;
    final static double PIXELS_PER_FOOT_RANGE = CANVAS_HEIGHT / MAX_RANGE_FEET;
    final static double FUNNEL_ANGLE_RADIANS = FUNNEL_ANGLE_DEGREES * (Math.PI / 180.0);

    final static int BBB_COMMAND_SET_LIGHTS_OFF = 1;
    final static int BBB_COMMAND_SET_LIGHTS_NORMAL = 2;
    final static int BBB_COMMAND_SET_LIGHTS_COLLECTING = 3;
    final static int BBB_COMMAND_SET_LIGHTS_GOT_BALL = 4;
    final static int BBB_COMMAND_SET_LIGHTS_SHOOTING = 5;

    public static String m_sCmd;

    public static Boolean m_isShuttingDown = false;
    public static JReceiver m_receiver = null;
    public static GraphicsContext m_gc = null;

    public static void drawChangingParts()
    {
        if (m_gc != null)
        {
            if (m_receiver.m_currentTargetInfo.getIsBlueFound() != 0)
            {
                m_gc.setStroke(Color.WHITE);
                m_gc.setLineWidth(2);
                double angleInRadians = m_receiver.m_currentTargetInfo.getAngleFromStraightAheadToBlue() * (Math.PI / 180.0);
                int pixelsNearFar_X = (int) (m_receiver.m_currentTargetInfo.getDistanceToBlue() * PIXELS_PER_FOOT_RANGE);
                int pixelsRightLeft_Y = (int) (pixelsNearFar_X * Math.tan(angleInRadians));
                m_gc.strokeLine((CANVAS_WIDTH / 2), CANVAS_HEIGHT - 30, (CANVAS_WIDTH / 2) + pixelsRightLeft_Y, CANVAS_HEIGHT - pixelsNearFar_X);
            }
        }
    }

    public static void drawFixedParts()
    {
        if (m_gc != null)
        {
            m_gc.setFill(Color.GRAY);
            m_gc.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
            m_gc.setStroke(Color.WHITE);
            m_gc.setLineWidth(5);

            // Draw horizontal line with a gap
            m_gc.strokeLine(20, CANVAS_HEIGHT - 30, (CANVAS_WIDTH / 2) - (GAP_SIZE_PIXELS / 2), CANVAS_HEIGHT - 30);
            m_gc.strokeLine(CANVAS_WIDTH - 30, CANVAS_HEIGHT - 30, (CANVAS_WIDTH / 2) + (GAP_SIZE_PIXELS / 2), CANVAS_HEIGHT - 30);

            // Draw funnel lines
            double offsetFarFunnel;
            offsetFarFunnel = (GAP_SIZE_PIXELS / 2) + ((CANVAS_HEIGHT - 30) * Math.sin(FUNNEL_ANGLE_RADIANS / 2));
            m_gc.strokeLine((CANVAS_WIDTH / 2) + (GAP_SIZE_PIXELS / 2), CANVAS_HEIGHT - 30, (CANVAS_WIDTH / 2) + offsetFarFunnel, 0);
            m_gc.strokeLine((CANVAS_WIDTH / 2) - (GAP_SIZE_PIXELS / 2), CANVAS_HEIGHT - 30, (CANVAS_WIDTH / 2) - offsetFarFunnel, 0);
        }
    }

    private static class BbbReceiveLoop implements Runnable
    {

        @Override
        public void run()
        {
            try
            {
                m_receiver = new JReceiver();
                m_receiver.init();
                while (!m_isShuttingDown)
                {
                    if (!m_receiver.m_initOK)
                    {
                        System.out.println("JReceiver client did not initialize properly -- make sure server is running");
                        // Pause for 4 seconds
                        Thread.sleep(4000);
                    }
                    else
                    {
                        try
                        {
                            m_receiver.m_out.println("GET");
                            String textInput;
                            String result;
                            Integer iCount = 0;
                            while ((textInput = m_receiver.m_in.readLine()) != null)
                            {
                                m_receiver.m_currentTargetInfo.initTargetInfoFromText(textInput);
                                Platform.runLater(new Runnable()
                                {
                                    @Override
                                    public void run()
                                    {
                                        drawFixedParts();
                                        drawChangingParts();
                                    }
                                });
                                iCount++;
                                result = iCount.toString();
                                result += " -- ";
                                result += textInput;
                                System.out.println(result);
                            }
                        }
                        catch (IOException ex)
                        {
                            Logger.getLogger(JReceiver.class.getName()).log(Level.SEVERE, null, ex);
                        }
                    }
                }
            }
            catch (InterruptedException e)
            {
                System.out.println("JReceiver thread terminated unexpectedly");
            }
            System.out.println("JReceiver thread has stopped");
        }
    };

    public void sendBbbCommand(int cmd)
    {
        Integer i = new Integer(cmd);
        m_sCmd = i.toString();
        m_sCmd += "\n";
        m_receiver.m_out.println(m_sCmd);
    }

    public static void main(String[] args)
    {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage)
    {
        Thread t = new Thread(new BbbReceiveLoop());
        t.start();

        primaryStage.setTitle("Drawing Operations Test");
        primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>()
        {
            public void handle(WindowEvent event)
            {
                m_isShuttingDown = true;
            }
        });
        Group root = new Group();

        Canvas canvas = new Canvas(CANVAS_WIDTH, CANVAS_HEIGHT);
        MenuBar menuBar = new MenuBar();
        m_gc = canvas.getGraphicsContext2D();
        drawFixedParts();
        root.getChildren().add(canvas);

        Menu menu1 = new Menu("Set Lights");
        menuBar.getMenus().add(menu1);

        MenuItem menuItem_0 = new MenuItem("Off");
        menuItem_0.setOnAction(new EventHandler<ActionEvent>()
        {
            @Override
            public void handle(ActionEvent t)
            {
                sendBbbCommand(BBB_COMMAND_SET_LIGHTS_OFF);
            }
        });

        MenuItem menuItem_1 = new MenuItem("Normal");
        menuItem_1.setOnAction(new EventHandler<ActionEvent>()
        {
            @Override
            public void handle(ActionEvent t)
            {
                sendBbbCommand(BBB_COMMAND_SET_LIGHTS_NORMAL);
            }
        });

        MenuItem menuItem_2 = new MenuItem("Collecting");
        menuItem_2.setOnAction(new EventHandler<ActionEvent>()
        {
            @Override
            public void handle(ActionEvent t)
            {
                sendBbbCommand(BBB_COMMAND_SET_LIGHTS_COLLECTING);
            }
        });

        MenuItem menuItem_3 = new MenuItem("Got Ball");
        menuItem_3.setOnAction(new EventHandler<ActionEvent>()
        {
            @Override
            public void handle(ActionEvent t)
            {
                sendBbbCommand(BBB_COMMAND_SET_LIGHTS_GOT_BALL);
            }
        });

        MenuItem menuItem_4 = new MenuItem("Shooting");
        menuItem_4.setOnAction(new EventHandler<ActionEvent>()
        {
            @Override
            public void handle(ActionEvent t)
            {
                sendBbbCommand(BBB_COMMAND_SET_LIGHTS_SHOOTING);
            }
        });
        menu1.getItems().addAll(menuItem_0, menuItem_1, menuItem_2, menuItem_3, menuItem_4);

        root.getChildren().add(menuBar);
        primaryStage.setScene(new Scene(root));
        primaryStage.show();
    }
}
