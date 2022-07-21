// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;


public class Robot extends TimedRobot {
  public Robot(){
    super(0.05);
  }

  private double mag, mag2;
  private double seno, seno2;
  private int pov, pov2;    
  private double spd = 0.6;
  private double mL = 0,mR = 0;
  private double mE = 0, mA = 0;
  private double x1,y1,x2,y2;
  private boolean analogic1, analogic2;
  private double rt, lt, rt2, lt2;
  private boolean a,b,x,y;
  private boolean a2,b2,buttonX2;
  private boolean buttonY2, l = false;
  private boolean ltB,rtB;

  private double i = 0, j = 0;

  VictorSPX m_direita1 = new VictorSPX(1);
  VictorSPX m_direita2 = new VictorSPX(2);
  VictorSPX m_esquerda1 = new VictorSPX(3);
  VictorSPX m_esquerda2 = new VictorSPX(4);

  VictorSPX m_escalada = new VictorSPX(5);
  VictorSPX m_angulo = new VictorSPX(6);
  
  Joystick joystick1 = new Joystick(0);
  Joystick joystick2 = new Joystick(1);

  Thread m_visionThread;
  List<Double> vetpower = new ArrayList<>();
  String texto = "";
  int p,q;
  
  Accelerometer accelerometer = new BuiltInAccelerometer();

  @Override
  public void robotInit() {
    //Motores servos
    m_esquerda2.follow(m_esquerda1);
    m_direita2.follow(m_direita1);
    //Configura a força como 0, caso ela seja menor que 4%
    m_esquerda1.configNeutralDeadband(0.04);
    m_direita1.configNeutralDeadband(0.04);
    //Trava os motores, diminuindo a inércia
    m_esquerda1.setNeutralMode(NeutralMode.Brake);
    m_direita1.setNeutralMode(NeutralMode.Brake);
    //imagem();
  }

  @Override
  public void teleopPeriodic()  {
    // Atribuição de valores as variaveis
    x1 = joystick1.getRawAxis(0);
    y1 = -joystick1.getRawAxis(1);
    x2 = joystick1.getRawAxis(4);
    y2 = -joystick1.getRawAxis(5);
    pov = joystick1.getPOV();
    rt = joystick1.getRawAxis(3);
    lt = -joystick1.getRawAxis(2);
    a = joystick1.getRawButton(1);
    b = joystick1.getRawButton(2); 
    x = joystick1.getRawButton(3);
    y = joystick1.getRawButtonPressed(4);
    a2 = joystick2.getRawButton(1);
    b2 = joystick2.getRawButton(2); 
    buttonX2 = joystick2.getRawButton(2);
    buttonY2 = joystick2.getRawButton(4);
    
    if(buttonY2) l = !l;

    rt2 = joystick2.getRawAxis(3);
    lt2 = joystick2.getRawAxis(2);
    pov2 = joystick2.getPOV();

    // Calculo das magnitudes
    mag = Math.hypot(x1, y1);
    mag2 = Math.hypot(x2, y2);

    // Verificação do uso de botões
    buttonSe(x,a,b);
    // Reiniciação dos valores insignificantes
    resetAxis();
    // Verificação dos analogicos
    movementCalc();
    // Calculo dos triggers
    triggerCalc(rt,lt,x1);
    // Calculo do POV
    povCalc(pov); 
    // Inicialização da escalada
    escalada(rt2,lt2);
    angulo(pov2);
    
    lockMotors(l);
      //Execução das forças dos motores
      m_escalada.set(ControlMode.PercentOutput, mE);
      m_angulo.set(ControlMode.PercentOutput, mA);
      m_esquerda1.set(ControlMode.PercentOutput, mL);
      m_direita1.set(ControlMode.PercentOutput, - mR);
      
      // Exibição dos valores na simulação
      SmartDashboardF();

      if(y){
        p++;
      }
    
      if(p==1){
          SmartDashboard.putBoolean("Gravador:", true);
          texto += mL + "\n";
          texto += mR + "\n";
          q++;
          
      }else if(p<=2 && p!=1){
        SmartDashboard.putBoolean("Gravador:", false);
        try{
          saveGravacao();
        }catch (IOException e) {
          e.printStackTrace();
        }
      }
    }

  private void SmartDashboardF() {

      //Valores da Simulação referentes a Movimentação
      SmartDashboard.putNumber("ForcaMotor Esquerdo", mL);
      SmartDashboard.putNumber("ForçaMotor Direito", mR);
      SmartDashboard.putNumber("Magnitude Esquerda", mag);
      SmartDashboard.putNumber("Magnitude Direita", mag2);
      SmartDashboard.putNumber("Trigger Esquerdo", -lt);
      SmartDashboard.putNumber("Trigger Direito", rt);
      SmartDashboard.putString("Analogico ativo",analogicGate(analogic1,analogic2));
      SmartDashboard.putString("Trigger ativo", analogicGate(ltB,rtB));

      //Valores da Simulação referentes a Escalada
      SmartDashboard.putNumber("Trigger Direito 2", rt2);
      SmartDashboard.putNumber("Trigger Esquerdo 2", lt2);
      SmartDashboard.putNumber("Valor de I", i);
      SmartDashboard.putNumber("Valor de J", j);
      SmartDashboard.putNumber("Motor Escalada", mE);
      SmartDashboard.putNumber("Motor Angulacao", mA);

      //Valores da Simulação referentes a Gravação do Autônomo
      SmartDashboard.putNumber("Valor de Q", q);
      SmartDashboard.putNumber("Valor de P", p);

      SmartDashboard.putNumber("Acelerometro X", accelerometer.getX());
      SmartDashboard.putNumber("Acelerometro Y", accelerometer.getY());
      SmartDashboard.putNumber("Acelerometro Z", accelerometer.getZ());
  }

  private String analogicGate(boolean a, boolean b) {
    
    if(a) return "Esquerdo"; // Verificação do uso do componente esquerdo

    else if(b) return "Direito"; // Verificação do uso do componente direito

    else return "Nenhum"; // Verificação da inutilização dos dois componentes

  }

  private void resetAxis() {

    // Verificação de inatividade dos analogicos e reset dos eixos e magnitude
    if(mag < 0.055){
      x1 = 0;
      y1 = 0;
      mag = 0;
    }
    if(mag2 < 0.055){
      x2 = 0;
      y2 = 0;
      mag2 = 0;
    }
    // Verificação da inatividade de ambos analogicos
    if(mag < 0.1 && mag2 < 0.1 && rt==0 && lt == 0){
      mL = 0;
      mR = 0;
    }

  }

  //Função da movimentação por Gatilhos
  public void triggerCalc(double rt,double lt,double x){
    if(Math.abs(x) < 0.04) x = 0;

    if(rt != 0){
        rtB = true;
        ltB = false;
      if(x >= 0){
        SmartDashboard.putString("Condicao", "Trigger Direito");
        mL = rt * spd;
        mR = rt * (1 - x) * spd;
      }else if(x < 0){
        SmartDashboard.putString("Condicao", "Trigger Direito");
        mL = rt * (1 + x) * spd;
        mR = rt *spd;
      }
    }else if(lt != 0){
      ltB = true;
      rtB = false;
      if(x >= 0){
        SmartDashboard.putString("Condicao", "Trigger Esquerdo");
        mL = lt * (1 - x) * spd;
        mR = lt * spd;
      }else if(x < 0){
        SmartDashboard.putString("Condicao", "Trigger Esquerdo");
        mL = lt * spd;
        mR = lt * (1 + x) * spd;
      }

    }
  }

  //Função da Movimentação pelo Analógico 1
  public void quadCalc(double y, double x) {
      seno = y / mag;
      // Quadrante 1  
    if(y >= 0 && x >= 0){
      SmartDashboard.putString("Condicao", "1 Q. Esquerdo");
      mR = (2 * seno - 1) * mag * spd; // Varia
      mL = mag * spd; // Constante
      // Quadrante 2
    }else if(y >= 0 && x <= 0){
      SmartDashboard.putString("Condicao", "2 Q. Esquerdo");
      mR = mag * spd; // Constante
      mL = (2 * seno - 1) * mag * spd; // Varia
      // Quadrante 3
    }else if(y < 0 && x < 0){
      SmartDashboard.putString("Condicao", "3 Q. Esquerdo");
      mR = (2 * seno + 1) * mag * spd; // Varia
      mL = -mag * spd; // Constante
      // Quadrante 4
    }else if(y < 0 && x >= 0){
      SmartDashboard.putString("Condicao", "4 Q. Esquerdo");
      mR = -mag * spd; // Constante
      mL = (2 * seno + 1) * mag * spd; // Varia
  }
}

  //Função da movimentação por POV (Botões digitais)
  public void povCalc(int pov){
    // Calculo do POV
    switch(pov){

      case 0:
      SmartDashboard.putString("Condicao", "POV");
      mR = 0.25;
      mL = 0.25;
      break;

      case 45:
      SmartDashboard.putString("Condicao", "POV");
      mR = 0;
      mL = 0.25;
      break;

      case 90:
      SmartDashboard.putString("Condicao", "POV");
      mR = -0.25;
      mL = 0.25;
      break;

      case 135:
      SmartDashboard.putString("Condicao", "POV");
      mR = -0.25;
      mL = 0;
      break;

      case 180:
      SmartDashboard.putString("Condicao", "POV");
      mR = -0.25;
      mL = -0.25;
      break;

      case 225:
      SmartDashboard.putString("Condicao", "POV");
      mR = 0;
      mL = -0.25;
      break;

      case 270:
      SmartDashboard.putString("Condicao", "POV");
      mR = 0.25;
      mL = -0.25;
      break;

      case 315:
      SmartDashboard.putString("Condicao", "POV");
      mR = 0.25;
      mL = 0; 
      break;
      
    }
      
      
  }

  //Função para verificar qual analógico está sendo usado e movimentar a partir dele  
  private void movementCalc(){

    //Verificação do analogico esquerdo
    if(minMethod(mag) != 0){
      analogic1 = true;
      analogic2 = false;
      // Calculo dos quadrantes
      quadCalc(y1, x1);
    } 

    else if(minMethod(mag2) != 0){
      // Calculo dos quadrantes
      reverseQuadCalc(); 
      analogic1 = false;
      analogic2 = true;
    }

    else{
      analogic1 = false;
      analogic2 = false;
    }
}

  //Função do Analógico 2 (Movimentação reversa)
  private void reverseQuadCalc() {
      seno2 = y2 / mag2;
        // Quadrante 1 
      if(y2 >= 0 && x2 >= 0){
        SmartDashboard.putString("Condicao", "1 Q. Direito");
        mR = - mag2 * spd;
        mL = (- 2 * seno2 + 1) * mag2 * spd;
        // Quadrante 2
      }else if(y2 >= 0 && x2 < 0){
        SmartDashboard.putString("Condicao", "2 Q. Direito");
        mR = (- 2 * seno2 + 1) * mag2 * spd;
        mL = - mag2 * spd;
        // Quadrante 3
      }else if(y2 < 0 && x2 < 0){
        SmartDashboard.putString("Condicao", "3 Q. Direito");
        mR = mag2 * spd;
        mL = (-2 * seno2 - 1) * mag2 * spd;
        // Quadrante 4
      }else if(y2 < 0 && x2 >= 0){
        SmartDashboard.putString("Condicao", "4 Q. Direito");
        mR = (-2 * seno2 - 1) * mag2 * spd;
        mL = mag2 * spd;
      }
  }

  //Função dos botões
  private double buttonSe(boolean x,boolean a, boolean b){
    
    if (x)
      spd = 0.6;
      
    else if(a)
      spd = 0.45;

    else if(b)
      spd = 0.3;

      SmartDashboard.putNumber("Velocidade", spd);

      return spd;
  }

  //Função para resetar o valor de qualquer variável para 0  
  private double minMethod(double a){

    SmartDashboard.putNumber("A", a);

    if(Math.abs(a) <= 0.055*spd) return 0;

    else return a;
    
    }

  //Função que trava os motores caso Y2 seja pressionado
  private void lockMotors(boolean y){
    if(y){
      mR = 0;
      mL = 0;
    }
  }

  //Função que controla a escalada
  private double escalada(double rt, double lt){

    spd = buttonSe(buttonX2,a2,b2);

    if(rt > 0){
      mE = 1;
      i = i + spd;
      if(i >= 60){
        i = 60;
        mE = 0;
      }
    }else if(lt > 0 && i > 0){
      mE = -1;
      i = i - spd;
    }else if(i < 0){
      i = 0;
    }else{
      mE =0;
    }
    
    return mE * spd;
  }

  //Função que controla a angulação do braço
  private double angulo(int pov){

    if(pov == 0){
      mA = 1;
      j++;
      if(j >= 60){
        j = 60;
        mA = 0;
      }
    }else if(pov == 180 && j > 0){
      mA = -1;
      j--;
    }else if(j < 0){
      j = 0;
    }else{
      mA = 0;
    }
      return mA;
    }

    //Função para uso de câmera
    private void imagem(){
      m_visionThread =
      new Thread(
          () -> {
            // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
            // Set the resolution
            camera.setResolution(640, 480);
  
            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
  
            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();
  
            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
              // Tell the CvSink to grab a frame from the camera and put it
              // in the source mat.  If there is an error notify the output.
              if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
              }
              // Put a rectangle on the image
              Imgproc.rectangle(
                  mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
              // Give the output stream a new image to display
              outputStream.putFrame(mat);
            }
          });
      m_visionThread.setDaemon(true);
      m_visionThread.start();
    }

    @Override
  public void autonomousInit() {

  for (int i = 0; i < vetpower.size(); i++) {
    vetpower.remove(0);
  }

  try {
    SmartDashboard.putString("load Gravacao ", "Começando");
    loadGravacao();
    SmartDashboard.putString("load Gravacao ", "funcionando");
  } catch (IOException e) {
    SmartDashboard.putString("load Gravacao ", "Erro");
    e.printStackTrace();
  }
   p=0;
 }

 @Override
  public void autonomousPeriodic() {
    if(p<vetpower.size()){
      m_esquerda1.set(ControlMode.PercentOutput, vetpower.get(p));
      SmartDashboard.putNumber("motor esquerdo", vetpower.get(p));
      p++;
      m_direita1.set(ControlMode.PercentOutput, -vetpower.get(p));
      SmartDashboard.putNumber("motor direito", vetpower.get(p));
      p++;
    }

 }
  private void saveGravacao() throws IOException {

    Path path = Path.of("C:/Users/SESI LEGO/Documents/ArquivoFRC.txt");

    if(Files.notExists(path)) Files.createFile(path);

    Writer out = new FileWriter("C:/Users/SESI LEGO/Documents/ArquivoFRC.txt");
    out.write("");
    out.flush();
    out.close();

    Files.writeString(path,texto);
}

  private void loadGravacao() throws IOException {

    Path path = Path.of("/home/ArquivoFRC.txt");

    List<String> strings = Files.readAllLines(path);

    for (String texto : strings) {
        vetpower.add(Double.parseDouble(texto)); 
    }
  }
}
