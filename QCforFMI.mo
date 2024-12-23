package QC
  model Quadcopter
    QC.Chasis chasis annotation(
      Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_ClockWise rPU_ClockWise annotation(
      Placement(visible = true, transformation(origin = {0, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_ClockWise rPU_ClockWise1 annotation(
      Placement(visible = true, transformation(origin = {0, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.Sensors sensors annotation(
      Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}})));
    QC.Controller controller annotation(
      Placement(transformation(origin = {-36, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealInput F_Wind[3] annotation(
      Placement(transformation(origin = {88, 40}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {100, 40}, extent = {{20, -20}, {-20, 20}})));
    Modelica.Blocks.Interfaces.RealInput T_Wind[3] annotation(
      Placement(transformation(origin = {88, -42}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {100, -40}, extent = {{20, -20}, {-20, 20}})));
    RPU_CounterClockWise rPU_CounterClockWise annotation(
      Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}})));
  RPU_CounterClockWise rPU_CounterClockWise1 annotation(
      Placement(transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(controller.RF, rPU_ClockWise.u) annotation(
      Line(points = {{-27, 7}, {-19, 7}, {-19, 42}, {-10, 42}}, color = {0, 0, 127}));
    connect(controller.LB, rPU_ClockWise1.u) annotation(
      Line(points = {{-27, -2}, {-15, -2}, {-15, -14}, {-10, -14}}, color = {0, 0, 127}));
    connect(rPU_ClockWise.Torque, chasis.T_RF) annotation(
      Line(points = {{10, 38}, {22, 38}, {22, 6}, {26, 6}}, color = {0, 0, 127}, thickness = 0.5));
    connect(rPU_ClockWise.Force, chasis.F_RF) annotation(
      Line(points = {{10, 46}, {24, 46}, {24, 8}, {26, 8}}, color = {0, 0, 127}, thickness = 0.5));
    connect(rPU_ClockWise1.Force, chasis.F_LB) annotation(
      Line(points = {{10, -10}, {18, -10}, {18, -2}, {26, -2}}, color = {0, 0, 127}, thickness = 0.5));
    connect(rPU_ClockWise1.Torque, chasis.T_LB) annotation(
      Line(points = {{10, -18}, {20, -18}, {20, -4}, {26, -4}}, color = {0, 0, 127}, thickness = 0.5));
    connect(sensors.Position, controller.position) annotation(
      Line(points = {{-10, -65}, {-60, -65}, {-60, 6}, {-46, 6}}, color = {0, 0, 127}, thickness = 0.5));
    connect(sensors.Attitude, controller.attitude) annotation(
      Line(points = {{-10, -75}, {-54, -75}, {-54, -4}, {-46, -4}}, color = {0, 0, 127}, thickness = 0.5));
    connect(chasis.ActualPosition, sensors.RealPosition) annotation(
      Line(points = {{46, -6}, {50, -6}, {50, -66}, {10, -66}}, color = {0, 0, 127}, thickness = 0.5));
    connect(chasis.ActualAngle, sensors.RealAngle) annotation(
      Line(points = {{46, -8}, {52, -8}, {52, -75}, {10, -75}}, color = {0, 0, 127}, thickness = 0.5));
    connect(F_Wind, chasis.F_Wind) annotation(
      Line(points = {{88, 40}, {60, 40}, {60, 6}, {46, 6}}, color = {0, 0, 127}, thickness = 0.5));
    connect(T_Wind, chasis.T_Wind) annotation(
      Line(points = {{88, -42}, {60, -42}, {60, 2}, {46, 2}}, color = {0, 0, 127}, thickness = 0.5));
    connect(rPU_CounterClockWise.Force, chasis.F_LF) annotation(
      Line(points = {{10, -36}, {22, -36}, {22, -6}, {26, -6}}, color = {0, 0, 127}, thickness = 0.5));
    connect(rPU_CounterClockWise.Torque, chasis.T_LF) annotation(
      Line(points = {{10, -44}, {24, -44}, {24, -8}, {26, -8}}, color = {0, 0, 127}, thickness = 0.5));
    connect(controller.LF, rPU_CounterClockWise.u) annotation(
      Line(points = {{-26, -6}, {-20, -6}, {-20, -40}, {-10, -40}}, color = {0, 0, 127}));
    connect(controller.RB, rPU_CounterClockWise1.u) annotation(
      Line(points = {{-26, 2}, {-16, 2}, {-16, 16}, {-10, 16}}, color = {0, 0, 127}));
  connect(rPU_CounterClockWise1.Force, chasis.F_RB) annotation(
      Line(points = {{10, 20}, {20, 20}, {20, 4}, {26, 4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rPU_CounterClockWise1.Torque, chasis.T_RB) annotation(
      Line(points = {{10, 12}, {18, 12}, {18, 2}, {26, 2}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"));
  end Quadcopter;

  model Chasis
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(m = 0.01, r = {0.1, 0, 0.1}) annotation(
      Placement(transformation(origin = {-30, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape1(m = 0.01, r = {0.1, 0, -0.1}) annotation(
      Placement(transformation(origin = {-30, 20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape2(m = 0.01, r = {-0.1, 0, -0.1}) annotation(
      Placement(transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape3(m = 0.01, r = {-0.1, 0, 0.1}) annotation(
      Placement(transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Body body(animation = true, m = 0.3, sphereDiameter = 0.04) annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    model PhysicalMotorModel
      Modelica.Mechanics.MultiBody.Parts.Body body1(m = 0.05, sphereDiameter = 0.05) annotation(
        Placement(transformation(origin = {0, 46}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Forces.WorldTorque torque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
        Placement(transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
        Placement(transformation(origin = {-30, 20}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
      Modelica.Blocks.Interfaces.RealInput Force[3] annotation(
        Placement(transformation(origin = {-100, 40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, 40}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput Torque[3] annotation(
        Placement(transformation(origin = {-100, -40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, -40}, extent = {{-20, -20}, {20, 20}})));
    equation
      connect(body1.frame_a, frame_b) annotation(
        Line(points = {{0, 36}, {0, 0}, {100, 0}}, color = {95, 95, 95}));
      connect(Force, worldForce2.force) annotation(
        Line(points = {{-100, 40}, {-60, 40}, {-60, 20}, {-42, 20}}, color = {0, 0, 127}, thickness = 0.5));
      connect(Torque, torque.torque) annotation(
        Line(points = {{-100, -40}, {-60, -40}, {-60, -20}, {-42, -20}}, color = {0, 0, 127}, thickness = 0.5));
      connect(worldForce2.frame_b, body1.frame_a) annotation(
        Line(points = {{-20, 20}, {0, 20}, {0, 36}}, color = {95, 95, 95}));
      connect(torque.frame_b, body1.frame_a) annotation(
        Line(points = {{-20, -20}, {0, -20}, {0, 36}}, color = {95, 95, 95}));
    end PhysicalMotorModel;

    PhysicalMotorModel PhysicalMotorModel_RF annotation(
      Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}})));
    PhysicalMotorModel physicalMotorModel_RB annotation(
      Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));
    PhysicalMotorModel physicalMotorModel_LB annotation(
      Placement(transformation(origin = {-70, -20}, extent = {{-10, -10}, {10, 10}})));
    PhysicalMotorModel physicalMotorModel_LF annotation(
      Placement(transformation(origin = {-70, -60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealInput F_RF[3] annotation(
      Placement(transformation(origin = {-101, 67}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, 79}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput T_RF[3] annotation(
      Placement(transformation(origin = {-101, 53}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, 61}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput F_RB[3] annotation(
      Placement(transformation(origin = {-101, 27}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, 37}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput T_RB[3] annotation(
      Placement(transformation(origin = {-101, 13}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, 19}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput F_LB[3] annotation(
      Placement(transformation(origin = {-101, -13}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, -21}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput T_LB[3] annotation(
      Placement(transformation(origin = {-101, -27}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, -39}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput F_LF[3] annotation(
      Placement(transformation(origin = {-101, -53}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, -61}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Blocks.Interfaces.RealInput T_LF[3] annotation(
      Placement(transformation(origin = {-101, -67}, extent = {{-7, -7}, {7, 7}}), iconTransformation(origin = {-99, -79}, extent = {{-9, -9}, {9, 9}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
      Placement(transformation(origin = {70, -80}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(transformation(origin = {70, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
    Modelica.Blocks.Interfaces.RealOutput ActualPosition[3] annotation(
      Placement(transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput ActualAngle[3] annotation(
      Placement(transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealInput F_Wind[3] annotation(
      Placement(transformation(origin = {100, 80}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {100, 68}, extent = {{20, -20}, {-20, 20}}, rotation = -0)));
    Modelica.Blocks.Interfaces.RealInput T_Wind[3] annotation(
      Placement(transformation(origin = {100, 40}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {100, 28}, extent = {{20, -20}, {-20, 20}}, rotation = -0)));
    Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
      Placement(transformation(origin = {50, 80}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque torque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
      Placement(transformation(origin = {50, 40}, extent = {{10, -10}, {-10, 10}})));
  equation
    connect(body.frame_a, bodyShape3.frame_b) annotation(
      Line(points = {{-10, 0}, {-14, 0}, {-14, -60}, {-20, -60}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape2.frame_b) annotation(
      Line(points = {{-10, 0}, {-14, 0}, {-14, -20}, {-20, -20}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape.frame_b) annotation(
      Line(points = {{-10, 0}, {-14, 0}, {-14, 59}, {-20, 59}, {-20, 60}}, color = {95, 95, 95}));
    connect(bodyShape1.frame_b, body.frame_a) annotation(
      Line(points = {{-20, 20}, {-14, 20}, {-14, 0}, {-10, 0}}, color = {95, 95, 95}));
    connect(PhysicalMotorModel_RF.frame_b, bodyShape.frame_a) annotation(
      Line(points = {{-60, 60}, {-40, 60}}, color = {95, 95, 95}));
    connect(physicalMotorModel_RB.frame_b, bodyShape1.frame_a) annotation(
      Line(points = {{-60, 20}, {-40, 20}}, color = {95, 95, 95}));
    connect(physicalMotorModel_LB.frame_b, bodyShape2.frame_a) annotation(
      Line(points = {{-60, -20}, {-40, -20}}, color = {95, 95, 95}));
    connect(physicalMotorModel_LF.frame_b, bodyShape3.frame_a) annotation(
      Line(points = {{-60, -60}, {-40, -60}}, color = {95, 95, 95}));
    connect(F_RF, PhysicalMotorModel_RF.Force) annotation(
      Line(points = {{-100, 68}, {-90, 68}, {-90, 64}, {-80, 64}}, color = {0, 0, 127}, thickness = 0.5));
    connect(T_RF, PhysicalMotorModel_RF.Torque) annotation(
      Line(points = {{-100, 54}, {-90, 54}, {-90, 56}, {-80, 56}}, color = {0, 0, 127}, thickness = 0.5));
    connect(F_RB, physicalMotorModel_RB.Force) annotation(
      Line(points = {{-100, 28}, {-90, 28}, {-90, 24}, {-80, 24}}, color = {0, 0, 127}, thickness = 0.5));
    connect(T_RB, physicalMotorModel_RB.Torque) annotation(
      Line(points = {{-100, 14}, {-90, 14}, {-90, 16}, {-80, 16}}, color = {0, 0, 127}, thickness = 0.5));
    connect(F_LB, physicalMotorModel_LB.Force) annotation(
      Line(points = {{-100, -12}, {-90, -12}, {-90, -16}, {-80, -16}}, color = {0, 0, 127}, thickness = 0.5));
    connect(T_LB, physicalMotorModel_LB.Torque) annotation(
      Line(points = {{-100, -26}, {-90, -26}, {-90, -24}, {-80, -24}}, color = {0, 0, 127}, thickness = 0.5));
    connect(F_LF, physicalMotorModel_LF.Force) annotation(
      Line(points = {{-100, -52}, {-92, -52}, {-92, -56}, {-80, -56}}, color = {0, 0, 127}, thickness = 0.5));
    connect(T_LF, physicalMotorModel_LF.Torque) annotation(
      Line(points = {{-100, -66}, {-92, -66}, {-92, -64}, {-80, -64}}, color = {0, 0, 127}, thickness = 0.5));
    connect(body.frame_a, absolutePosition.frame_a) annotation(
      Line(points = {{-10, 0}, {44, 0}, {44, -60}, {60, -60}}, color = {95, 95, 95}));
    connect(body.frame_a, absoluteAngles.frame_a) annotation(
      Line(points = {{-10, 0}, {44, 0}, {44, -80}, {60, -80}}, color = {95, 95, 95}));
    connect(absolutePosition.r, ActualPosition) annotation(
      Line(points = {{82, -60}, {100, -60}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteAngles.angles, ActualAngle) annotation(
      Line(points = {{82, -80}, {100, -80}}, color = {0, 0, 127}, thickness = 0.5));
    connect(T_Wind, torque.torque) annotation(
      Line(points = {{100, 40}, {62, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(F_Wind, worldForce2.force) annotation(
      Line(points = {{100, 80}, {62, 80}}, color = {0, 0, 127}, thickness = 0.5));
    connect(torque.frame_b, body.frame_a) annotation(
      Line(points = {{40, 40}, {-10, 40}, {-10, 0}}, color = {95, 95, 95}));
    connect(worldForce2.frame_b, body.frame_a) annotation(
      Line(points = {{40, 80}, {-10, 80}, {-10, 0}}, color = {95, 95, 95}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {170, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-70, 74}, extent = {{-14, 12}, {14, -12}}, textString = "RF"), Text(origin = {-69, 31}, extent = {{-13, 13}, {13, -13}}, textString = "RB"), Text(origin = {-69, -26}, extent = {{-11, 14}, {11, -14}}, textString = "LB"), Text(origin = {-69, -74}, extent = {{-11, 12}, {11, -12}}, textString = "LF"), Text(origin = {73, 48}, extent = {{-13, 12}, {13, -12}}, textString = "W"), Text(origin = {71, -48}, extent = {{13, 12}, {-13, -12}}, textString = "S"), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "F")}));
  end Chasis;

  model RotaryPropultionUnit
    Modelica.Blocks.Math.MatrixGain matrixGain(K = [1, 0, 0; 0, 1, 0; 0, 0, 1]) annotation(
      Placement(visible = true, transformation(origin = {38, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-104, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 100, uMin = 0) annotation(
      Placement(visible = true, transformation(origin = {-52, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {-34, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain(k = 0.01) annotation(
      Placement(visible = true, transformation(origin = {-22, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Product product1 annotation(
      Placement(visible = true, transformation(origin = {-34, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const1(k = 2) annotation(
      Placement(visible = true, transformation(origin = {-90, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const2(k = 0.1) annotation(
      Placement(visible = true, transformation(origin = {-90, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-34, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.Multiplex mux(n = 3) annotation(
      Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.Multiplex mux1(n = 3) annotation(
      Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-34, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Force[3] annotation(
      Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput Torque[3] annotation(
      Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(limiter.u, u) annotation(
      Line(points = {{-64, 68}, {-76, 68}, {-76, 0}, {-104, 0}}, color = {0, 0, 127}));
    connect(limiter.y, gain.u) annotation(
      Line(points = {{-41, 68}, {-34, 68}}, color = {0, 0, 127}));
    connect(gain.y, product.u1) annotation(
      Line(points = {{-10, 68}, {0, 68}, {0, 32}, {-56, 32}, {-56, -54}, {-46, -54}}, color = {0, 0, 127}));
    connect(gain.y, product1.u1) annotation(
      Line(points = {{-10, 68}, {0, 68}, {0, 32}, {-56, 32}, {-56, -24}, {-46, -24}}, color = {0, 0, 127}));
    connect(const1.y, product1.u2) annotation(
      Line(points = {{-79, -36}, {-46, -36}}, color = {0, 0, 127}));
    connect(const2.y, product.u2) annotation(
      Line(points = {{-78, -66}, {-46, -66}}, color = {0, 0, 127}));
    connect(mux1.y, matrixGain.u) annotation(
      Line(points = {{12, -60}, {26, -60}}, color = {0, 0, 127}, thickness = 0.5));
    connect(constant1.y, mux.u[1]) annotation(
      Line(points = {{-22, -2}, {-14, -2}, {-14, -30}, {-10, -30}}, color = {0, 0, 127}));
    connect(constant1.y, mux1.u[1]) annotation(
      Line(points = {{-22, -2}, {-14, -2}, {-14, -60}, {-10, -60}}, color = {0, 0, 127}));
    connect(product1.y, mux.u[2]) annotation(
      Line(points = {{-22, -30}, {-10, -30}}, color = {0, 0, 127}));
    connect(product.y, mux1.u[2]) annotation(
      Line(points = {{-22, -60}, {-10, -60}}, color = {0, 0, 127}));
    connect(const.y, mux1.u[3]) annotation(
      Line(points = {{-22, -84}, {-20, -84}, {-20, -60}, {-10, -60}}, color = {0, 0, 127}));
    connect(const.y, mux.u[3]) annotation(
      Line(points = {{-22, -84}, {-20, -84}, {-20, -30}, {-10, -30}}, color = {0, 0, 127}));
    connect(mux.y, Force) annotation(
      Line(points = {{12, -30}, {80, -30}, {80, 40}, {100, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(matrixGain.y, Torque) annotation(
      Line(points = {{50, -60}, {80, -60}, {80, -40}, {100, -40}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Diagram,
  Icon(graphics = {Rectangle(origin = {-100, 0}, fillColor = {170, 255, 127}, fillPattern = FillPattern.Solid, extent = {{0, 100}, {201, -100}}), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "M")}));
  end RotaryPropultionUnit;

  model RPU_ClockWise
    extends QC.RotaryPropultionUnit;
  equation

  end RPU_ClockWise;

  model RPU_CounterClockWise
    extends QC.RotaryPropultionUnit(matrixGain.K = [1, 0, 0; 0, -1, 0; 0, 0, 1]);
  equation

  end RPU_CounterClockWise;

  model Sensors
    Modelica.Blocks.Interfaces.RealOutput Position[3] annotation(
      Placement(transformation(origin = {-106, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-102, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealOutput Attitude[3] annotation(
      Placement(transformation(origin = {-106, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-104, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput RealPosition[3] annotation(
      Placement(transformation(origin = {100, 40}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {102, 40}, extent = {{20, -20}, {-20, 20}}, rotation = -0)));
    Modelica.Blocks.Interfaces.RealInput RealAngle[3] annotation(
      Placement(transformation(origin = {100, -40}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {98, -54}, extent = {{20, -20}, {-20, 20}})));
  equation
    connect(RealPosition, Position) annotation(
      Line(points = {{100, 40}, {-106, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(RealAngle, Attitude) annotation(
      Line(points = {{100, -40}, {-106, -40}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
      Icon(graphics = {Rectangle(fillColor = {170, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "S")}));
end Sensors;

  model Controller
    Modelica.Blocks.Interfaces.RealInput position[3] annotation(
      Placement(transformation(origin = {-100, 58}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
      Placement(transformation(origin = {-102, -90}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput RF annotation(
      Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {98, 70}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput RB annotation(
      Placement(transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput LB annotation(
      Placement(transformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput LF annotation(
      Placement(transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}})));

    model MotorMixingAlgorithm
      Modelica.Blocks.Interfaces.RealOutput RF annotation(
        Placement(visible = true, transformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput RB annotation(
        Placement(visible = true, transformation(origin = {100, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput LB annotation(
        Placement(visible = true, transformation(origin = {100, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput LF annotation(
        Placement(visible = true, transformation(origin = {100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput T annotation(
        Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput R annotation(
        Placement(visible = true, transformation(origin = {-100, 18}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 28}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput P annotation(
        Placement(visible = true, transformation(origin = {-100, -34}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -28}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Y annotation(
        Placement(visible = true, transformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Math.Add3 add3 annotation(
        Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add1 annotation(
        Placement(visible = true, transformation(origin = {70, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add2 annotation(
        Placement(visible = true, transformation(origin = {70, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add4 annotation(
        Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add3 add31(k1 = -1, k3 = -1) annotation(
        Placement(visible = true, transformation(origin = {30, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add3 add32(k2 = -1, k3 = -1) annotation(
        Placement(visible = true, transformation(origin = {30, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add3 add33(k1 = -1, k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(T, add.u1) annotation(
        Line(points = {{-100, 60}, {-76, 60}, {-76, 94}, {52, 94}, {52, 76}, {58, 76}}, color = {0, 0, 127}));
      connect(T, add1.u1) annotation(
        Line(points = {{-100, 60}, {-76, 60}, {-76, 94}, {52, 94}, {52, 34}, {58, 34}}, color = {0, 0, 127}));
      connect(T, add2.u1) annotation(
        Line(points = {{-100, 60}, {-76, 60}, {-76, 94}, {52, 94}, {52, -22}, {58, -22}}, color = {0, 0, 127}));
      connect(T, add4.u1) annotation(
        Line(points = {{-100, 60}, {-76, 60}, {-76, 94}, {52, 94}, {52, -64}, {58, -64}}, color = {0, 0, 127}));
      connect(add3.y, add.u2) annotation(
        Line(points = {{42, 70}, {46, 70}, {46, 64}, {58, 64}}, color = {0, 0, 127}));
      connect(add31.y, add1.u2) annotation(
        Line(points = {{42, 28}, {46, 28}, {46, 22}, {58, 22}}, color = {0, 0, 127}));
      connect(add32.y, add2.u2) annotation(
        Line(points = {{42, -28}, {46, -28}, {46, -34}, {58, -34}}, color = {0, 0, 127}));
      connect(add33.y, add4.u2) annotation(
        Line(points = {{42, -70}, {46, -70}, {46, -76}, {58, -76}}, color = {0, 0, 127}));
      connect(R, add3.u1) annotation(
        Line(points = {{-100, 18}, {-60, 18}, {-60, 78}, {18, 78}}, color = {0, 0, 127}));
      connect(R, add31.u1) annotation(
        Line(points = {{-100, 18}, {-60, 18}, {-60, 36}, {18, 36}}, color = {0, 0, 127}));
      connect(R, add32.u1) annotation(
        Line(points = {{-100, 18}, {-60, 18}, {-60, -20}, {18, -20}}, color = {0, 0, 127}));
      connect(R, add33.u1) annotation(
        Line(points = {{-100, 18}, {-60, 18}, {-60, -62}, {18, -62}}, color = {0, 0, 127}));
      connect(P, add3.u2) annotation(
        Line(points = {{-100, -34}, {-40, -34}, {-40, 70}, {18, 70}}, color = {0, 0, 127}));
      connect(P, add31.u2) annotation(
        Line(points = {{-100, -34}, {-40, -34}, {-40, 28}, {18, 28}}, color = {0, 0, 127}));
      connect(P, add32.u2) annotation(
        Line(points = {{-100, -34}, {-40, -34}, {-40, -28}, {18, -28}}, color = {0, 0, 127}));
      connect(P, add33.u2) annotation(
        Line(points = {{-100, -34}, {-40, -34}, {-40, -70}, {18, -70}}, color = {0, 0, 127}));
      connect(Y, add33.u3) annotation(
        Line(points = {{-100, -80}, {-20, -80}, {-20, -78}, {18, -78}}, color = {0, 0, 127}));
      connect(Y, add32.u3) annotation(
        Line(points = {{-100, -80}, {-20, -80}, {-20, -36}, {18, -36}}, color = {0, 0, 127}));
      connect(Y, add31.u3) annotation(
        Line(points = {{-100, -80}, {-20, -80}, {-20, 20}, {18, 20}}, color = {0, 0, 127}));
      connect(Y, add3.u3) annotation(
        Line(points = {{-100, -80}, {-20, -80}, {-20, 62}, {18, 62}}, color = {0, 0, 127}));
      connect(add.y, RF) annotation(
        Line(points = {{82, 70}, {100, 70}}, color = {0, 0, 127}));
      connect(add1.y, RB) annotation(
        Line(points = {{82, 28}, {100, 28}}, color = {0, 0, 127}));
      connect(add2.y, LB) annotation(
        Line(points = {{82, -28}, {100, -28}}, color = {0, 0, 127}));
      connect(add4.y, LF) annotation(
        Line(points = {{82, -70}, {100, -70}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(origin = {-70, 80}, extent = {{10, -8}, {-10, 8}}, textString = "T"), Text(origin = {-72, 28}, extent = {{-8, 8}, {8, -8}}, textString = "R"), Text(origin = {-70, -28}, extent = {{-8, 8}, {8, -8}}, textString = "P"), Text(origin = {-70, -80}, extent = {{-8, 8}, {8, -8}}, textString = "Y"), Text(origin = {74, -60}, extent = {{-8, 8}, {8, -8}}, textString = "LF"), Text(origin = {74, 20}, extent = {{-8, 8}, {8, -8}}, textString = "RB"), Text(origin = {74, -20}, extent = {{-8, 8}, {8, -8}}, textString = "LB"), Text(origin = {74, 60}, extent = {{10, -8}, {-10, 8}}, textString = "RF"), Rectangle(extent = {{-100, 100}, {100, -100}})}));
    end MotorMixingAlgorithm;

    QC.Controller.MotorMixingAlgorithm motorMixingAlgorithm annotation(
      Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant HoverThrustConst(k = 66.2) annotation(
      Placement(visible = true, transformation(origin = {20, 78}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
      Placement(visible = true, transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID pid(Td = 1.5, Ti = 1000, controllerType = Modelica.Blocks.Types.SimpleController.PID, initType = Modelica.Blocks.Types.Init.NoInit, k = 7, yMax = 10, yMin = -10, y_start = 0) annotation(
      Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant des_hight(k = 0.5) annotation(
      Placement(visible = true, transformation(origin = {-30, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add annotation(
      Placement(visible = true, transformation(origin = {44, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID RollPID(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 30, wd = 0.5, wp = 0.001, yMax = 10, yMin = -10) annotation(
      Placement(visible = true, transformation(origin = {40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID PitchPID(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = -30, wd = 0, yMax = 10, yMin = -10) annotation(
      Placement(visible = true, transformation(origin = {38, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID YawPID1(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 30, yMax = 10, yMin = -10) annotation(
      Placement(visible = true, transformation(origin = {38, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    model PositionDecoder
      Modelica.Blocks.Interfaces.RealInput X_abs annotation(
        Placement(visible = true, transformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Y_abs annotation(
        Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput X_des annotation(
        Placement(visible = true, transformation(origin = {-100, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Y_des annotation(
        Placement(visible = true, transformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Theta annotation(
        Placement(visible = true, transformation(origin = {0, -100}, extent = {{-20, 20}, {20, -20}}, rotation = 90), iconTransformation(origin = {0, -90}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
      parameter Real dTheta = 0.0;
      Real x_err_internal;
      Real y_err_internal;
      Real Theta_internal;
      Modelica.Blocks.Interfaces.RealOutput X_err annotation(
        Placement(visible = true, transformation(origin = {88, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Y_err annotation(
        Placement(visible = true, transformation(origin = {100, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      x_err_internal = X_des - X_abs;
      y_err_internal = Y_des - Y_abs;
      Theta_internal = Theta + dTheta;
      X_err = x_err_internal;
      Y_err = y_err_internal;
//X_err = cos(Theta_internal)*x_err_internal - sin(Theta_internal)*y_err_internal;
//Y_err = sin(Theta_internal)*x_err_internal + cos(Theta_internal)*y_err_internal;
      annotation(
        Icon(graphics = {Text(origin = {-32, -47}, extent = {{-34, 35}, {34, -35}}, textString = "Des"), Text(origin = {-29, 51}, extent = {{-33, 31}, {33, -31}}, textString = "Abs"), Text(origin = {45, 7}, extent = {{-33, 35}, {33, -35}}, textString = "Loc"), Text(origin = {42, -84}, extent = {{34, -20}, {-34, 20}}, textString = "Theta"), Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {85, 12}, extent = {{-3, 6}, {3, -6}}, textString = "x"), Text(origin = {85, -14}, extent = {{-3, 6}, {3, -6}}, textString = "y"), Text(origin = {-67, 70}, extent = {{-3, 6}, {3, -6}}, textString = "x"), Text(origin = {-69, 30}, extent = {{-3, 6}, {3, -6}}, textString = "y"), Text(origin = {-69, -28}, extent = {{-3, 6}, {3, -6}}, textString = "x"), Text(origin = {-69, -70}, extent = {{-3, 6}, {3, -6}}, textString = "y")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end PositionDecoder;

    QC.Controller.PositionDecoder positionDecoder annotation(
      Placement(visible = true, transformation(origin = {-44, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID limPID(Td = 2.5, controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 0.05, wd = 0.01, wp = 0.005, yMax = 0.05) annotation(
      Placement(transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Continuous.LimPID limPID1(Td = 2.5, controllerType = Modelica.Blocks.Types.SimpleController.PD, k = -0.05, wd = 0.01, wp = 0.005, yMax = 0.05) annotation(
      Placement(visible = true, transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant constant2(k = 0.0) annotation(
      Placement(transformation(origin = {-44, 50}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(motorMixingAlgorithm.RF, RF) annotation(
      Line(points = {{84, 6}, {86, 6}, {86, 60}, {100, 60}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.RB, RB) annotation(
      Line(points = {{84, 2}, {88, 2}, {88, 20}, {100, 20}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.LB, LB) annotation(
      Line(points = {{84, -2}, {88, -2}, {88, -20}, {100, -20}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.LF, LF) annotation(
      Line(points = {{84, -6}, {86, -6}, {86, -60}, {100, -60}}, color = {0, 0, 127}));
    connect(des_hight.y, pid.u_s) annotation(
      Line(points = {{-19, 90}, {-12, 90}}, color = {0, 0, 127}));
    connect(HoverThrustConst.y, add.u2) annotation(
      Line(points = {{27, 78}, {32, 78}}, color = {0, 0, 127}));
    connect(pid.y, add.u1) annotation(
      Line(points = {{11, 90}, {32, 90}}, color = {0, 0, 127}));
    connect(add.y, motorMixingAlgorithm.T) annotation(
      Line(points = {{55, 84}, {58, 84}, {58, 8}, {64, 8}}, color = {0, 0, 127}));
    connect(const1.y, RollPID.u_s) annotation(
      Line(points = {{21, -20}, {28, -20}}, color = {0, 0, 127}));
    connect(RollPID.y, motorMixingAlgorithm.R) annotation(
      Line(points = {{51, -20}, {52.5, -20}, {52.5, 3}, {64, 3}}, color = {0, 0, 127}));
    connect(constant1.y, positionDecoder.X_des) annotation(
      Line(points = {{-69, 0}, {-64, 0}, {-64, 20}, {-54, 20}}, color = {0, 0, 127}));
    connect(constant1.y, positionDecoder.Y_des) annotation(
      Line(points = {{-69, 0}, {-64, 0}, {-64, 16}, {-54, 16}}, color = {0, 0, 127}));
    connect(constant2.y, limPID.u_s) annotation(
      Line(points = {{-33, 50}, {-22, 50}}, color = {0, 0, 127}));
    connect(constant2.y, limPID1.u_s) annotation(
      Line(points = {{-33, 50}, {-26, 50}, {-26, 20}, {-22, 20}}, color = {0, 0, 127}));
    connect(positionDecoder.X_err, limPID.u_m) annotation(
      Line(points = {{-34, 24}, {-30, 24}, {-30, 38}, {-10, 38}}, color = {0, 0, 127}));
    connect(positionDecoder.Y_err, limPID1.u_m) annotation(
      Line(points = {{-34, 20}, {-30, 20}, {-30, 8}, {-10, 8}}, color = {0, 0, 127}));
    connect(PitchPID.y, motorMixingAlgorithm.P) annotation(
      Line(points = {{50, -50}, {56, -50}, {56, -2}, {64, -2}}, color = {0, 0, 127}));
    connect(YawPID1.y, motorMixingAlgorithm.Y) annotation(
      Line(points = {{50, -80}, {60, -80}, {60, -8}, {64, -8}}, color = {0, 0, 127}));
    connect(limPID.y, PitchPID.u_s) annotation(
      Line(points = {{1, 50}, {11, 50}, {11, -4}, {-8, -4}, {-8, -56}, {26, -56}, {26, -50}}, color = {0, 0, 127}));
    connect(limPID1.y, YawPID1.u_s) annotation(
      Line(points = {{2, 20}, {8, 20}, {8, -2}, {-14, -2}, {-14, -72}, {26, -72}, {26, -80}}, color = {0, 0, 127}));
    connect(position[1], positionDecoder.X_abs) annotation(
      Line(points = {{-100, 58}, {-100, 58.6875}, {-70, 58.6875}, {-70, 28}, {-54, 28}, {-54, 30}}, color = {0, 0, 127}));
    connect(position[2], pid.u_m) annotation(
      Line(points = {{-100, 58}, {-70, 58}, {-70, 68}, {0, 68}, {0, 78}}, color = {0, 0, 127}));
    connect(position[3], positionDecoder.Y_abs) annotation(
      Line(points = {{-100, 58}, {-100, 59}, {-72, 59}, {-72, 26}, {-54, 26}}, color = {0, 0, 127}));
    connect(attitude[1], YawPID1.u_m) annotation(
      Line(points = {{-102, -90}, {-90, -90}, {-90, -98}, {38, -98}, {38, -92}}, color = {0, 0, 127}));
    connect(attitude[2], RollPID.u_m) annotation(
      Line(points = {{-102, -90}, {-91.5, -90}, {-91.5, -88}, {-91, -88}, {-91, -84}, {-44, -84}, {-44, -36}, {40, -36}, {40, -32}}, color = {0, 0, 127}));
    connect(attitude[2], positionDecoder.Theta) annotation(
      Line(points = {{-102, -90}, {-91, -90}, {-91, -84}, {-44, -84}, {-44, 14}}, color = {0, 0, 127}));
    connect(attitude[3], PitchPID.u_m) annotation(
      Line(points = {{-102, -90}, {-36, -90}, {-36, -62}, {38, -62}}, color = {0, 0, 127}));
    annotation(
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
  Icon(graphics = {Rectangle(fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "C")}));
  end Controller;

  model Wind
    Modelica.Blocks.Sources.CombiTimeTable ForceLUT(table = [0, 0, 0, 0; 3, 0.002, 0.01, -0.001; 5, 0.01, 0.015, 0.01; 6, 0, 0.004, -0.02; 8, -0.005, 0.0, 0.001; 9, 0.01, -0.005, 0; 10, 0, 0, 0; 100, 0, 0, 0]) annotation(
      Placement(transformation(origin = {0, 40}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.CombiTimeTable TorqueLUT(table = [0, 0, 0, 0; 3, 0, -0.001, 0.001; 5, 0.001, 0, -0.001; 6, 0, 0.005, -0.002; 8, 0, 0.008, 0; 9, 0.001, -0.002, 0; 10, 0, 0, 0; 100, 0, 0, 0]) annotation(
      Placement(transformation(origin = {0, -40}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput F_Wind[3] annotation(
      Placement(transformation(origin = {-100, 40}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-100, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Interfaces.RealOutput T_Wind[3] annotation(
      Placement(transformation(origin = {-100, -40}, extent = {{10, -10}, {-10, 10}}), iconTransformation(origin = {-100, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  equation
    connect(ForceLUT.y, F_Wind) annotation(
      Line(points = {{-10, 40}, {-100, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(TorqueLUT.y, T_Wind) annotation(
      Line(points = {{-10, -40}, {-100, -40}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Documentation,
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end Wind;

  model MulticopterSystemContext
    Quadcopter quadcopter annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}})));
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(transformation(origin = {80, 60}, extent = {{-10, -10}, {10, 10}})));
    Wind wind annotation(
      Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(wind.F_Wind, quadcopter.F_Wind) annotation(
      Line(points = {{70, 4}, {10, 4}}, color = {0, 0, 127}, thickness = 0.5));
    connect(wind.T_Wind, quadcopter.T_Wind) annotation(
      Line(points = {{70, -4}, {10, -4}}, color = {0, 0, 127}, thickness = 0.5));
  end MulticopterSystemContext;
  annotation(
    uses(Modelica(version = "4.0.0"), Modelica_DeviceDrivers(version = "2.1.1")));
end QC;