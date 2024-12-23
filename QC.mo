package QC
  model Quadcopter
    QC.Chasis chasis annotation(
      Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_CounterClockWise rPU_CounterClockWise annotation(
      Placement(visible = true, transformation(origin = {0, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_CounterClockWise rPU_CounterClockWise1 annotation(
      Placement(visible = true, transformation(origin = {0, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_ClockWise rPU_ClockWise annotation(
      Placement(visible = true, transformation(origin = {0, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_ClockWise rPU_ClockWise1 annotation(
      Placement(visible = true, transformation(origin = {0, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.Sensors sensors annotation(
      Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}})));
    QC.Controller controller annotation(
      Placement(transformation(origin = {-36, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a Wind annotation(
      Placement(transformation(origin = {72, 4}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  equation
    connect(controller.RF, rPU_ClockWise.u) annotation(
      Line(points = {{-27, 7}, {-19, 7}, {-19, 42}, {-10, 42}}, color = {0, 0, 127}));
    connect(controller.RB, rPU_CounterClockWise.u) annotation(
      Line(points = {{-27, 2}, {-15, 2}, {-15, 14}, {-10, 14}}, color = {0, 0, 127}));
    connect(controller.LB, rPU_ClockWise1.u) annotation(
      Line(points = {{-27, -2}, {-15, -2}, {-15, -14}, {-10, -14}}, color = {0, 0, 127}));
    connect(controller.LF, rPU_CounterClockWise1.u) annotation(
      Line(points = {{-27, -6}, {-19, -6}, {-19, -42}, {-10, -42}}, color = {0, 0, 127}));
    connect(rPU_ClockWise.frame_b, chasis.RF_MotorMount) annotation(
      Line(points = {{10, 42}, {20, 42}, {20, 8}, {26, 8}}, color = {95, 95, 95}));
    connect(rPU_CounterClockWise.frame_b, chasis.RB_MotorMount) annotation(
      Line(points = {{10, 14}, {16, 14}, {16, 3}, {26, 3}}, color = {95, 95, 95}));
    connect(rPU_ClockWise1.frame_b, chasis.LB_MotorMount) annotation(
      Line(points = {{10, -14}, {16, -14}, {16, -3}, {26, -3}}, color = {95, 95, 95}));
    connect(rPU_CounterClockWise1.frame_b, chasis.LF_MotorMount) annotation(
      Line(points = {{10, -42}, {20, -42}, {20, -8}, {26, -8}}));
    connect(rPU_ClockWise.frame_b, chasis.RF_MotorMount) annotation(
      Line(points = {{10, 42}, {20, 42}, {20, 7}, {26, 7}, {26, 8}}, color = {95, 95, 95}));
    connect(rPU_CounterClockWise.frame_b, chasis.RB_MotorMount) annotation(
      Line(points = {{10, 14}, {16, 14}, {16, 2}, {26, 2}}, color = {95, 95, 95}));
    connect(rPU_ClockWise1.frame_b, chasis.LB_MotorMount) annotation(
      Line(points = {{10, -14}, {16, -14}, {16, -2}, {26, -2}}, color = {95, 95, 95}));
    connect(rPU_CounterClockWise1.frame_b, chasis.LF_MotorMount) annotation(
      Line(points = {{10, -42}, {20, -42}, {20, -8}, {26, -8}}, color = {95, 95, 95}));
    connect(chasis.WindEffects, Wind) annotation(
      Line(points = {{46, 4}, {72, 4}}, color = {95, 95, 95}));
    connect(sensors.Position, controller.position) annotation(
      Line(points = {{-10, -65}, {-60, -65}, {-60, 5}, {-46, 5}}, color = {0, 0, 127}, thickness = 0.5));
    connect(sensors.Attitude, controller.attitude) annotation(
      Line(points = {{-10, -75}, {-52, -75}, {-52, -5.375}, {-46, -5.375}, {-46, -5}}, color = {0, 0, 127}, thickness = 0.5));
    connect(chasis.SensorMount, sensors.frame_a) annotation(
      Line(points = {{46, -4}, {54, -4}, {54, -70}, {10, -70}}, color = {95, 95, 95}));
    annotation(
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"));
  end Quadcopter;

  model Chasis
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(m = 0.01, r = {0.1, 0, 0.1}) annotation(
      Placement(visible = true, transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape1(m = 0.01, r = {0.1, 0, -0.1}) annotation(
      Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape2(m = 0.01, r = {-0.1, 0, -0.1}) annotation(
      Placement(visible = true, transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape3(m = 0.01, r = {-0.1, 0, 0.1}) annotation(
      Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a RF_MotorMount annotation(
      Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 76}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a RB_MotorMount annotation(
      Placement(visible = true, transformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 28}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a LB_MotorMount annotation(
      Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, -26}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a LF_MotorMount annotation(
      Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, -76}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(animation = true, m = 0.3, sphereDiameter = 0.04) annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a SensorMount annotation(
      Placement(transformation(origin = {100, -50}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -50}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a WindEffects annotation(
      Placement(transformation(origin = {100, 50}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {98, 48}, extent = {{-16, -16}, {16, 16}})));
  equation
    connect(bodyShape.frame_a, RF_MotorMount) annotation(
      Line(points = {{-60, 60}, {-100, 60}}));
    connect(RB_MotorMount, bodyShape1.frame_a) annotation(
      Line(points = {{-100, 20}, {-60, 20}}));
    connect(bodyShape2.frame_a, LB_MotorMount) annotation(
      Line(points = {{-60, -20}, {-100, -20}}, color = {95, 95, 95}));
    connect(bodyShape3.frame_a, LF_MotorMount) annotation(
      Line(points = {{-60, -60}, {-100, -60}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape3.frame_b) annotation(
      Line(points = {{-10, 0}, {-20, 0}, {-20, -44}, {-19, -44}, {-19, -60}, {-40, -60}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape2.frame_b) annotation(
      Line(points = {{-10, 0}, {-26, 0}, {-26, -20}, {-40, -20}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape1.frame_b) annotation(
      Line(points = {{-10, 0}, {-25, 0}, {-25, 20}, {-28, 20}, {-28, 21}, {-34, 21}, {-34, 20.5}, {-40, 20.5}, {-40, 20}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape.frame_b) annotation(
      Line(points = {{-10, 0}, {-20, 0}, {-20, 61}, {-40, 61}, {-40, 60}}, color = {95, 95, 95}));
    connect(body.frame_a, SensorMount) annotation(
      Line(points = {{-10, 0}, {43, 0}, {43, -50}, {100, -50}}, color = {95, 95, 95}));
    connect(body.frame_a, WindEffects) annotation(
      Line(points = {{-10, 0}, {44, 0}, {44, 49}, {100, 49}, {100, 50}}, color = {95, 95, 95}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {170, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-70, 74}, extent = {{-14, 12}, {14, -12}}, textString = "RF"), Text(origin = {-69, 31}, extent = {{-13, 13}, {13, -13}}, textString = "RB"), Text(origin = {-69, -26}, extent = {{-11, 14}, {11, -14}}, textString = "LB"), Text(origin = {-69, -74}, extent = {{-11, 12}, {11, -12}}, textString = "LF"), Text(origin = {73, 48}, extent = {{-13, 12}, {13, -12}}, textString = "W"), Text(origin = {71, -48}, extent = {{13, 12}, {-13, -12}}, textString = "S"), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "F")}));
  end Chasis;

  model RotaryPropultionUnit
    Modelica.Mechanics.MultiBody.Parts.Body body(m = 0.05, sphereDiameter = 0.05) annotation(
      Placement(visible = true, transformation(origin = {4, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque torque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
      Placement(visible = true, transformation(origin = {70, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
      Placement(visible = true, transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
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
      Placement(transformation(origin = {-84, -36}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant const2(k = 0.1) annotation(
      Placement(transformation(origin = {-84, -66}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant const(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-34, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.Multiplex mux(n = 3) annotation(
      Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.Multiplex mux1(n = 3) annotation(
      Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-34, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(body.frame_a, frame_b) annotation(
      Line(points = {{14, 2}, {100, 2}, {100, 0}}));
    connect(worldForce2.frame_b, body.frame_a) annotation(
      Line(points = {{80, -30}, {80, -29.75}, {82, -29.75}, {82, -29.5}, {90, -29.5}, {90, 2}, {14, 2}}, color = {95, 95, 95}));
    connect(torque.frame_b, body.frame_a) annotation(
      Line(points = {{80, -60}, {80, -49.25}, {90, -49.25}, {90, 2.375}, {14, 2.375}, {14, 2}}, color = {95, 95, 95}));
    connect(torque.torque, matrixGain.y) annotation(
      Line(points = {{58, -60}, {49, -60}}, color = {0, 0, 127}, thickness = 0.5));
    connect(limiter.u, u) annotation(
      Line(points = {{-64, 68}, {-76, 68}, {-76, 0}, {-104, 0}}, color = {0, 0, 127}));
    connect(limiter.y, gain.u) annotation(
      Line(points = {{-41, 68}, {-34, 68}}, color = {0, 0, 127}));
    connect(gain.y, product.u1) annotation(
      Line(points = {{-10, 68}, {0, 68}, {0, 32}, {-56, 32}, {-56, -54}, {-46, -54}}, color = {0, 0, 127}));
    connect(gain.y, product1.u1) annotation(
      Line(points = {{-10, 68}, {0, 68}, {0, 32}, {-56, 32}, {-56, -24}, {-46, -24}}, color = {0, 0, 127}));
    connect(const1.y, product1.u2) annotation(
      Line(points = {{-73, -36}, {-46, -36}}, color = {0, 0, 127}));
    connect(const2.y, product.u2) annotation(
      Line(points = {{-73, -66}, {-46, -66}}, color = {0, 0, 127}));
    connect(mux.y, worldForce2.force) annotation(
      Line(points = {{12, -30}, {58, -30}}, color = {0, 0, 127}, thickness = 0.5));
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
    annotation(
      Icon(graphics = {Rectangle(fillColor = {170, 255, 127}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "M")}));
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
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
      Placement(visible = true, transformation(origin = {0, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(visible = true, transformation(origin = {0, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Position[3] annotation(
      Placement(transformation(origin = {-106, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-102, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealOutput Attitude[3] annotation(
      Placement(transformation(origin = {-106, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-104, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  equation
    connect(absolutePosition.frame_a, frame_a) annotation(
      Line(points = {{10, 40}, {40, 40}, {40, 0}, {100, 0}}));
    connect(absoluteAngles.frame_a, frame_a) annotation(
      Line(points = {{10, -40}, {40, -40}, {40, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(absolutePosition.r, Position) annotation(
      Line(points = {{-10, 40}, {-106, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteAngles.angles, Attitude) annotation(
      Line(points = {{-10, -40}, {-106, -40}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {170, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "S")}));
  end Sensors;

  model Controller
    Modelica.Blocks.Interfaces.RealInput position[3] annotation(
      Placement(transformation(origin = {-100, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
      Placement(transformation(origin = {-100, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput RF annotation(
      Placement(visible = true, transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput RB annotation(
      Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput LB annotation(
      Placement(visible = true, transformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput LF annotation(
      Placement(visible = true, transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

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
        Placement(transformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput Y annotation(
        Placement(transformation(origin = {-100, 28}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 28}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput P annotation(
        Placement(transformation(origin = {-100, -28}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, -28}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput R annotation(
        Placement(transformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}})));
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
        Line(points = {{-100, 80}, {-76, 80}, {-76, 94}, {52, 94}, {52, 76}, {58, 76}}, color = {0, 0, 127}));
      connect(T, add1.u1) annotation(
        Line(points = {{-100, 80}, {-76, 80}, {-76, 94}, {52, 94}, {52, 34}, {58, 34}}, color = {0, 0, 127}));
      connect(T, add2.u1) annotation(
        Line(points = {{-100, 80}, {-76, 80}, {-76, 94}, {52, 94}, {52, -22}, {58, -22}}, color = {0, 0, 127}));
      connect(T, add4.u1) annotation(
        Line(points = {{-100, 80}, {-76, 80}, {-76, 94}, {52, 94}, {52, -64}, {58, -64}}, color = {0, 0, 127}));
      connect(add3.y, add.u2) annotation(
        Line(points = {{42, 70}, {46, 70}, {46, 64}, {58, 64}}, color = {0, 0, 127}));
      connect(add31.y, add1.u2) annotation(
        Line(points = {{42, 28}, {46, 28}, {46, 22}, {58, 22}}, color = {0, 0, 127}));
      connect(add32.y, add2.u2) annotation(
        Line(points = {{42, -28}, {46, -28}, {46, -34}, {58, -34}}, color = {0, 0, 127}));
      connect(add33.y, add4.u2) annotation(
        Line(points = {{42, -70}, {46, -70}, {46, -76}, {58, -76}}, color = {0, 0, 127}));
      connect(Y, add3.u1) annotation(
        Line(points = {{-100, 28}, {-60, 28}, {-60, 78}, {18, 78}}, color = {0, 0, 127}));
      connect(Y, add31.u1) annotation(
        Line(points = {{-100, 28}, {-60, 28}, {-60, 36}, {18, 36}}, color = {0, 0, 127}));
      connect(Y, add32.u1) annotation(
        Line(points = {{-100, 28}, {-60, 28}, {-60, -20}, {18, -20}}, color = {0, 0, 127}));
      connect(Y, add33.u1) annotation(
        Line(points = {{-100, 28}, {-60, 28}, {-60, -62}, {18, -62}}, color = {0, 0, 127}));
      connect(P, add3.u2) annotation(
        Line(points = {{-100, -28}, {-40, -28}, {-40, 70}, {18, 70}}, color = {0, 0, 127}));
      connect(P, add31.u2) annotation(
        Line(points = {{-100, -28}, {-40, -28}, {-40, 28}, {18, 28}}, color = {0, 0, 127}));
      connect(P, add32.u2) annotation(
        Line(points = {{-100, -28}, {18, -28}}, color = {0, 0, 127}));
      connect(P, add33.u2) annotation(
        Line(points = {{-100, -28}, {-40, -28}, {-40, -70}, {18, -70}}, color = {0, 0, 127}));
      connect(R, add33.u3) annotation(
        Line(points = {{-100, -80}, {-41, -80}, {-41, -78}, {18, -78}}, color = {0, 0, 127}));
      connect(R, add32.u3) annotation(
        Line(points = {{-100, -80}, {-20, -80}, {-20, -36}, {18, -36}}, color = {0, 0, 127}));
      connect(R, add31.u3) annotation(
        Line(points = {{-100, -80}, {-20, -80}, {-20, 20}, {18, 20}}, color = {0, 0, 127}));
      connect(R, add3.u3) annotation(
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
        Icon(graphics = {Text(origin = {-70, 80}, extent = {{10, -8}, {-10, 8}}, textString = "T"), Text(origin = {-72, 28}, extent = {{-8, 8}, {8, -8}}, textString = "R"), Text(origin = {-70, -28}, extent = {{-8, 8}, {8, -8}}, textString = "P"), Text(origin = {-70, -80}, extent = {{-8, 8}, {8, -8}}, textString = "Y"), Text(origin = {74, -60}, extent = {{-8, 8}, {8, -8}}, textString = "LF"), Text(origin = {74, 20}, extent = {{-8, 8}, {8, -8}}, textString = "RB"), Text(origin = {74, -20}, extent = {{-8, 8}, {8, -8}}, textString = "LB"), Text(origin = {74, 60}, extent = {{10, -8}, {-10, 8}}, textString = "RF"), Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 3}, extent = {{-47, 27}, {47, -27}}, textString = "MMA")}));
    end MotorMixingAlgorithm;

    QC.Controller.MotorMixingAlgorithm motorMixingAlgorithm annotation(
      Placement(transformation(origin = {32, 0}, extent = {{-20, -20}, {20, 20}})));

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
        Placement(transformation(origin = {100, 14}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 12}, extent = {{-10, -10}, {10, 10}})));
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

    model LowLevelControl
      Modelica.Blocks.Interfaces.RealInput position[3] annotation(
        Placement(transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 50}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
        Placement(transformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, -50}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealOutput T annotation(
        Placement(transformation(origin = {100, 80}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput Y annotation(
        Placement(transformation(origin = {102, 28}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 28}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput P annotation(
        Placement(transformation(origin = {100, -32}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -28}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput R annotation(
        Placement(transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
        Placement(transformation(origin = {48, 6}, extent = {{-10, -10}, {10, 10}})));

      model LateralContol
        Modelica.Blocks.Interfaces.RealOutput P annotation(
          Placement(transformation(origin = {106, 50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput R annotation(
          Placement(transformation(origin = {106, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, -50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput position[3] annotation(
          Placement(transformation(origin = {-108, 70}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 50}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
          Placement(transformation(origin = {-108, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, -50}, extent = {{-20, -20}, {20, 20}})));
        QC.Controller.PositionDecoder positionDecoder annotation(
          Placement(transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
          Placement(transformation(origin = {-64, -56}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Continuous.LimPID limPID(Td = 2.5, controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 0.05, wd = 0.01, wp = 0.005, yMax = 0.05) annotation(
          Placement(transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Continuous.LimPID limPID1(Td = 2.5, controllerType = Modelica.Blocks.Types.SimpleController.PD, k = -0.05, wd = 0.01, wp = 0.005, yMax = 0.05) annotation(
          Placement(transformation(origin = {30, -50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Constant constant2(k = 0.0) annotation(
          Placement(transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(constant1.y, positionDecoder.X_des) annotation(
          Line(points = {{-53, -56}, {-52, -56}, {-52, -53}, {-39, -53}}, color = {0, 0, 127}));
        connect(constant1.y, positionDecoder.Y_des) annotation(
          Line(points = {{-53, -56}, {-52, -56}, {-52, -57}, {-39, -57}}, color = {0, 0, 127}));
        connect(constant2.y, limPID.u_s) annotation(
          Line(points = {{-19, 50}, {18, 50}}, color = {0, 0, 127}));
        connect(constant2.y, limPID1.u_s) annotation(
          Line(points = {{-19, 50}, {0, 50}, {0, -50}, {18, -50}}, color = {0, 0, 127}));
        connect(positionDecoder.X_err, limPID.u_m) annotation(
          Line(points = {{-20, -49}, {-10, -49}, {-10, 38}, {30, 38}}, color = {0, 0, 127}));
        connect(positionDecoder.Y_err, limPID1.u_m) annotation(
          Line(points = {{-20, -51}, {-10, -51}, {-10, -70}, {30, -70}, {30, -62}}, color = {0, 0, 127}));
        connect(attitude[2], positionDecoder.Theta) annotation(
          Line(points = {{-108, -50}, {-86, -50}, {-86, -90}, {-30, -90}, {-30, -58}}, color = {0, 0, 127}));
        connect(position[1], positionDecoder.X_abs) annotation(
          Line(points = {{-108, 70}, {-44, 70}, {-44, -42}, {-40, -42}}, color = {0, 0, 127}));
        connect(position[3], positionDecoder.Y_abs) annotation(
          Line(points = {{-108, 70}, {-44, 70}, {-44, -46}, {-40, -46}}, color = {0, 0, 127}));
        connect(limPID.y, P) annotation(
          Line(points = {{42, 50}, {106, 50}}, color = {0, 0, 127}));
        connect(limPID1.y, R) annotation(
          Line(points = {{42, -50}, {106, -50}}, color = {0, 0, 127}));
      end LateralContol;

      LateralContol lateralContol annotation(
        Placement(transformation(origin = {-11, -71}, extent = {{-25, -25}, {25, 25}})));

      model ThrustControll
        Modelica.Blocks.Interfaces.RealOutput T annotation(
          Placement(transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput position[3] annotation(
          Placement(transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Sources.Constant HoverThrustConst(k = 66.2) annotation(
          Placement(transformation(origin = {26, -6}, extent = {{-6, -6}, {6, 6}})));
        Modelica.Blocks.Continuous.LimPID pid(Td = 1.5, Ti = 1000, controllerType = Modelica.Blocks.Types.SimpleController.PID, initType = Modelica.Blocks.Types.Init.NoInit, k = 7, yMax = 10, yMin = -10, y_start = 0) annotation(
          Placement(transformation(origin = {0, 6}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Constant des_hight(k = 0.5) annotation(
          Placement(transformation(origin = {-30, 6}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Add add annotation(
          Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(des_hight.y, pid.u_s) annotation(
          Line(points = {{-19, 6}, {-12, 6}}, color = {0, 0, 127}));
        connect(HoverThrustConst.y, add.u2) annotation(
          Line(points = {{32.6, -6}, {37.6, -6}}, color = {0, 0, 127}));
        connect(pid.y, add.u1) annotation(
          Line(points = {{11, 6}, {38, 6}}, color = {0, 0, 127}));
        connect(position[2], pid.u_m) annotation(
          Line(points = {{-108, 0}, {-60, 0}, {-60, -20}, {0, -20}, {0, -6}}, color = {0, 0, 127}));
        connect(add.y, T) annotation(
          Line(points = {{62, 0}, {106, 0}}, color = {0, 0, 127}));
      end ThrustControll;

      ThrustControll thrustControll annotation(
        Placement(transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}})));

      model YawControll
        Modelica.Blocks.Interfaces.RealOutput Y annotation(
          Placement(transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
          Placement(transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(transformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
        Modelica.Blocks.Continuous.LimPID YawPID(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 30, wd = 0.5, wp = 0.001, yMax = 10, yMin = -10) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
      equation
        connect(YawPID.y, Y) annotation(
          Line(points = {{12, 0}, {102, 0}}, color = {0, 0, 127}));
  connect(attitude[2], YawPID.u_m) annotation(
          Line(points = {{-100, 0}, {-60, 0}, {-60, -42}, {0, -42}, {0, -12}}, color = {0, 0, 127}));
  connect(u, YawPID.u_s) annotation(
          Line(points = {{0, -100}, {0, -58}, {-36, -58}, {-36, 0}, {-12, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {6, 3}, extent = {{-74, 39}, {74, -39}}, textString = "Yaw Ctrl")}));
      end YawControll;

      model PitchControll
        Modelica.Blocks.Interfaces.RealOutput P annotation(
          Placement(transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
          Placement(transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(transformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
        Modelica.Blocks.Continuous.LimPID PitchPID(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = -30, wd = 0, yMax = 10, yMin = -10) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
      equation
        connect(PitchPID.y, P) annotation(
          Line(points = {{12, 0}, {102, 0}}, color = {0, 0, 127}));
  connect(attitude[3], PitchPID.u_m) annotation(
          Line(points = {{-100, 0}, {-50, 0}, {-50, -22}, {0, -22}, {0, -12}}, color = {0, 0, 127}));
  connect(u, PitchPID.u_s) annotation(
          Line(points = {{0, -100}, {0, -48}, {-30, -48}, {-30, 0}, {-12, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {6, 3}, extent = {{-74, 39}, {74, -39}}, textString = "Pitch Ctrl")}));
      end PitchControll;

      model RollControll
        Modelica.Blocks.Interfaces.RealOutput R annotation(
          Placement(transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput attitude[3] annotation(
          Placement(transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(transformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
        Modelica.Blocks.Continuous.LimPID RollPID1(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 30, yMax = 10, yMin = -10) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
      equation
        connect(RollPID1.y, R) annotation(
          Line(points = {{12, 0}, {102, 0}}, color = {0, 0, 127}));
  connect(attitude[1], RollPID1.u_m) annotation(
          Line(points = {{-100, 0}, {-54, 0}, {-54, -34}, {0, -34}, {0, -12}}, color = {0, 0, 127}));
  connect(u, RollPID1.u_s) annotation(
          Line(points = {{0, -100}, {0, -50}, {-32, -50}, {-32, 0}, {-12, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {6, 3}, extent = {{-74, 39}, {74, -39}}, textString = "Roll Ctrl")}));
      end RollControll;

      YawControll yawControll annotation(
        Placement(transformation(origin = {70, 28}, extent = {{-10, -10}, {10, 10}})));
      PitchControll pitchControll annotation(
        Placement(transformation(origin = {70, -32}, extent = {{-10, -10}, {10, 10}})));
      RollControll rollControll annotation(
        Placement(transformation(origin = {70, -80}, extent = {{-10, -10}, {10, 10}})));
    equation
      connect(position, lateralContol.position) annotation(
        Line(points = {{-100, 60}, {-50, 60}, {-50, -58.5}, {-36, -58.5}}, color = {0, 0, 127}, thickness = 0.5));
      connect(thrustControll.T, T) annotation(
        Line(points = {{80, 80}, {100, 80}}, color = {0, 0, 127}));
      connect(position, thrustControll.position) annotation(
        Line(points = {{-100, 60}, {-50, 60}, {-50, 80}, {60, 80}}, color = {0, 0, 127}, thickness = 0.5));
      connect(yawControll.Y, Y) annotation(
        Line(points = {{80, 28}, {102, 28}}, color = {0, 0, 127}));
      connect(const1.y, yawControll.u) annotation(
        Line(points = {{60, 6}, {70, 6}, {70, 18}}, color = {0, 0, 127}));
      connect(attitude, yawControll.attitude) annotation(
        Line(points = {{-100, -60}, {-60, -60}, {-60, 28}, {60, 28}}, color = {0, 0, 127}, thickness = 0.5));
      connect(pitchControll.P, P) annotation(
        Line(points = {{80, -32}, {100, -32}}, color = {0, 0, 127}));
      connect(attitude, pitchControll.attitude) annotation(
        Line(points = {{-100, -60}, {-60, -60}, {-60, -32}, {60, -32}}, color = {0, 0, 127}, thickness = 0.5));
      connect(rollControll.R, R) annotation(
        Line(points = {{80, -80}, {100, -80}}, color = {0, 0, 127}));
      connect(attitude, rollControll.attitude) annotation(
        Line(points = {{-100, -60}, {-60, -60}, {-60, -32}, {50, -32}, {50, -80}, {60, -80}}, color = {0, 0, 127}, thickness = 0.5));
      connect(lateralContol.P, pitchControll.u) annotation(
        Line(points = {{14.5, -58.5}, {70, -58.5}, {70, -42}}, color = {0, 0, 127}));
      connect(lateralContol.R, rollControll.u) annotation(
        Line(points = {{14.5, -83.5}, {14.5, -82.75}, {40.5, -82.75}, {40.5, -96}, {70, -96}, {70, -90}}, color = {0, 0, 127}));
      connect(attitude, lateralContol.attitude) annotation(
        Line(points = {{-100, -60}, {-60, -60}, {-60, -83.5}, {-36, -83.5}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, -1}, extent = {{-74, 31}, {74, -31}}, textString = "LLC")}),
        Diagram(coordinateSystem(extent = {{-120, 100}, {120, -100}})));
    end LowLevelControl;

    LowLevelControl lowLevelControl annotation(
      Placement(transformation(origin = {-28, 0}, extent = {{-20, -20}, {20, 20}})));
  equation
    connect(motorMixingAlgorithm.RF, RF) annotation(
      Line(points = {{52, 12}, {70, 12}, {70, 60}, {100, 60}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.RB, RB) annotation(
      Line(points = {{52, 4}, {80, 4}, {80, 20}, {100, 20}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.LB, LB) annotation(
      Line(points = {{52, -4}, {80, -4}, {80, -20}, {100, -20}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.LF, LF) annotation(
      Line(points = {{52, -12}, {70, -12}, {70, -60}, {100, -60}}, color = {0, 0, 127}));
    connect(lowLevelControl.T, motorMixingAlgorithm.T) annotation(
      Line(points = {{-8, 14}, {9, 14}, {9, 16}, {12, 16}}, color = {0, 0, 127}));
    connect(lowLevelControl.Y, motorMixingAlgorithm.Y) annotation(
      Line(points = {{-8, 5.6}, {12, 5.6}}, color = {0, 0, 127}));
    connect(lowLevelControl.P, motorMixingAlgorithm.P) annotation(
      Line(points = {{-8, -5.6}, {12, -5.6}}, color = {0, 0, 127}));
    connect(position, lowLevelControl.position) annotation(
      Line(points = {{-100, 50}, {-60, 50}, {-60, 10}, {-48, 10}}, color = {0, 0, 127}, thickness = 0.5));
    connect(attitude, lowLevelControl.attitude) annotation(
      Line(points = {{-100, -50}, {-60, -50}, {-60, -10}, {-48, -10}}, color = {0, 0, 127}, thickness = 0.5));
    connect(lowLevelControl.R, motorMixingAlgorithm.R) annotation(
      Line(points = {{-8, -14}, {12, -14}, {12, -16}}, color = {0, 0, 127}));
    annotation(
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"),
      Diagram(coordinateSystem(extent = {{-120, 80}, {120, -80}})),
      Icon(graphics = {Rectangle(fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-46, 52}, {46, -52}}, textString = "C")}));
  end Controller;

  model Wind
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
      Placement(visible = true, transformation(origin = {-50, 14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.CombiTimeTable ForceLUT(table = [0, 0, 0, 0; 3, 0.002, 0.01, -0.001; 5, 0.01, 0.015, 0.01; 6, 0, 0.004, -0.02; 8, -0.005, 0.0, 0.001; 9, 0.01, -0.005, 0; 10, 0, 0, 0; 100, 0, 0, 0]) annotation(
      Placement(visible = true, transformation(origin = {30, 14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque torque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
      Placement(visible = true, transformation(origin = {-50, -14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.CombiTimeTable TorqueLUT(table = [0, 0, 0, 0; 3, 0, -0.001, 0.001; 5, 0.001, 0, -0.001; 6, 0, 0.005, -0.002; 8, 0, 0.008, 0; 9, 0.001, -0.002, 0; 10, 0, 0, 0; 100, 0, 0, 0]) annotation(
      Placement(visible = true, transformation(origin = {30, -14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(worldForce2.frame_b, frame_b) annotation(
      Line(points = {{-60, 14}, {-80, 14}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(torque.frame_b, frame_b) annotation(
      Line(points = {{-60, -14}, {-80, -14}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(TorqueLUT.y, torque.torque) annotation(
      Line(points = {{20, -14}, {-38, -14}}, color = {0, 0, 127}, thickness = 0.5));
    connect(ForceLUT.y, worldForce2.force) annotation(
      Line(points = {{20, 14}, {-38, 14}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Documentation,
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"),
      Diagram(graphics = {Text(origin = {51, -10}, extent = {{-11, 8}, {11, -8}}, textString = "[x, y, z]")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end Wind;

  model MulticopterSystemContext
    Quadcopter quadcopter annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}})));
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(transformation(origin = {80, 60}, extent = {{-10, -10}, {10, 10}})));
    Wind wind annotation(
      Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(quadcopter.Wind, wind.frame_b) annotation(
      Line(points = {{10, 0}, {70, 0}}, color = {95, 95, 95}));
  end MulticopterSystemContext;
  annotation(
    uses(Modelica(version = "4.0.0"), Modelica_DeviceDrivers(version = "2.1.1")));
end QC;