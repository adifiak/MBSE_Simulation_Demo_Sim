package QC
  model Quadcopter
    QC.Chasis chasis annotation(
      Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_CounterClockWise rPU_CounterClockWise annotation(
      Placement(visible = true, transformation(origin = {24, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_CounterClockWise rPU_CounterClockWise1 annotation(
      Placement(visible = true, transformation(origin = {24, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {80, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_ClockWise rPU_ClockWise annotation(
      Placement(visible = true, transformation(origin = {24, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.RPU_ClockWise rPU_ClockWise1 annotation(
      Placement(visible = true, transformation(origin = {24, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.Sensors sensors annotation(
      Placement(visible = true, transformation(origin = {24, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QC.Controller controller annotation(
      Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Wind wind annotation(
      Placement(visible = true, transformation(origin = {80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(controller.RF, rPU_ClockWise.u) annotation(
      Line(points = {{-21, 7}, {-11, 7}, {-11, 55}, {13, 55}}, color = {0, 0, 127}));
    connect(controller.RB, rPU_CounterClockWise.u) annotation(
      Line(points = {{-21, 2}, {-1, 2}, {-1, 20}, {13, 20}}, color = {0, 0, 127}));
    connect(controller.LB, rPU_ClockWise1.u) annotation(
      Line(points = {{-21, -1.8}, {-1, -1.8}, {-1, -23.8}, {13, -23.8}}, color = {0, 0, 127}));
    connect(controller.LF, rPU_CounterClockWise1.u) annotation(
      Line(points = {{-21, -6}, {-11, -6}, {-11, -64}, {13, -64}}, color = {0, 0, 127}));
    connect(sensors.x, controller.x) annotation(
      Line(points = {{14, -81}, {-80, -81}, {-80, 9}, {-40, 9}}, color = {0, 0, 127}));
    connect(sensors.y, controller.y) annotation(
      Line(points = {{14, -83}, {-82, -83}, {-82, 5}, {-40, 5}}, color = {0, 0, 127}));
    connect(sensors.z, controller.z) annotation(
      Line(points = {{14, -85}, {-84, -85}, {-84, 1}, {-40, 1}}, color = {0, 0, 127}));
    connect(sensors.a, controller.a) annotation(
      Line(points = {{14, -91}, {-90, -91}, {-90, -3}, {-40, -3}}, color = {0, 0, 127}));
    connect(sensors.b, controller.b) annotation(
      Line(points = {{14, -93}, {-92, -93}, {-92, -7}, {-40, -7}}, color = {0, 0, 127}));
    connect(sensors.c, controller.c) annotation(
      Line(points = {{14, -95}, {-94, -95}, {-94, -11}, {-40, -11}}, color = {0, 0, 127}));
    connect(rPU_ClockWise.frame_b, chasis.frame_RF) annotation(
      Line(points = {{34, 56}, {44, 56}, {44, 8}, {50, 8}}, color = {95, 95, 95}));
    connect(rPU_CounterClockWise.frame_b, chasis.frame_a_RB) annotation(
      Line(points = {{34, 20}, {38, 20}, {38, 2}, {50, 2}}, color = {95, 95, 95}));
    connect(rPU_ClockWise1.frame_b, chasis.frame_a_LB) annotation(
      Line(points = {{34, -24}, {38, -24}, {38, -2}, {50, -2}}, color = {95, 95, 95}));
    connect(rPU_CounterClockWise1.frame_b, chasis.frame_a_LF) annotation(
      Line(points = {{34, -64}, {44, -64}, {44, -8}, {50, -8}}));
    connect(chasis.frame_a4, sensors.frame_a) annotation(
      Line(points = {{70, 0}, {80, 0}, {80, -88}, {34, -88}}, color = {95, 95, 95}));
    connect(wind.frame_b, chasis.frame_a4) annotation(
      Line(points = {{70, 40}, {60, 40}, {60, 20}, {80, 20}, {80, 0}, {70, 0}}, color = {95, 95, 95}));
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
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_RF annotation(
      Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 76}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_RB annotation(
      Placement(visible = true, transformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 28}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_LB annotation(
      Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, -26}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_LF annotation(
      Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, -76}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(animation = true, m = 0.3, sphereDiameter = 0.04) annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a4 annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
    connect(bodyShape.frame_a, frame_RF) annotation(
      Line(points = {{-60, 60}, {-100, 60}}));
    connect(frame_a_RB, bodyShape1.frame_a) annotation(
      Line(points = {{-100, 20}, {-60, 20}}));
    connect(bodyShape2.frame_a, frame_a_LB) annotation(
      Line(points = {{-60, -20}, {-100, -20}}, color = {95, 95, 95}));
    connect(bodyShape3.frame_a, frame_a_LF) annotation(
      Line(points = {{-60, -60}, {-100, -60}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape3.frame_b) annotation(
      Line(points = {{-10, 0}, {-20, 0}, {-20, -44}, {-19, -44}, {-19, -60}, {-40, -60}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape2.frame_b) annotation(
      Line(points = {{-10, 0}, {-26, 0}, {-26, -20}, {-40, -20}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape1.frame_b) annotation(
      Line(points = {{-10, 0}, {-25, 0}, {-25, 20}, {-28, 20}, {-28, 21}, {-34, 21}, {-34, 20.5}, {-40, 20.5}, {-40, 20}}, color = {95, 95, 95}));
    connect(body.frame_a, bodyShape.frame_b) annotation(
      Line(points = {{-10, 0}, {-20, 0}, {-20, 61}, {-40, 61}, {-40, 60}}, color = {95, 95, 95}));
    connect(body.frame_a, frame_a4) annotation(
      Line(points = {{-10, 0}, {100, 0}}, color = {95, 95, 95}));
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
      Line(points = {{-79, -36}, {-46, -36}}, color = {0, 0, 127}));
    connect(const2.y, product.u2) annotation(
      Line(points = {{-78, -66}, {-46, -66}}, color = {0, 0, 127}));
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
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {-96, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput x annotation(
      Placement(visible = true, transformation(origin = {-94, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput z annotation(
      Placement(visible = true, transformation(origin = {-96, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput a annotation(
      Placement(visible = true, transformation(origin = {-98, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput b annotation(
      Placement(visible = true, transformation(origin = {-98, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput c annotation(
      Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.DeMultiplex demux(n = 3) annotation(
      Placement(visible = true, transformation(origin = {-40, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.DeMultiplex demux1(n = 3) annotation(
      Placement(visible = true, transformation(origin = {-40, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(absolutePosition.frame_a, frame_a) annotation(
      Line(points = {{10, 40}, {40, 40}, {40, 0}, {100, 0}}));
    connect(absoluteAngles.frame_a, frame_a) annotation(
      Line(points = {{10, -40}, {40, -40}, {40, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(absolutePosition.r, demux.u) annotation(
      Line(points = {{-10, 40}, {-28, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteAngles.angles, demux1.u) annotation(
      Line(points = {{-10, -40}, {-28, -40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(demux1.y[1], a) annotation(
      Line(points = {{-50, -40}, {-70, -40}, {-70, -22}, {-98, -22}}, color = {0, 0, 127}));
    connect(demux1.y[2], b) annotation(
      Line(points = {{-50, -40}, {-98, -40}}, color = {0, 0, 127}));
    connect(demux1.y[3], c) annotation(
      Line(points = {{-50, -40}, {-70, -40}, {-70, -60}, {-100, -60}}, color = {0, 0, 127}));
    connect(demux.y[1], x) annotation(
      Line(points = {{-50, 40}, {-68, 40}, {-68, 62}, {-94, 62}}, color = {0, 0, 127}));
    connect(demux.y[2], y) annotation(
      Line(points = {{-50, 40}, {-96, 40}}, color = {0, 0, 127}));
    connect(demux.y[3], z) annotation(
      Line(points = {{-50, 40}, {-68, 40}, {-68, 20}, {-96, 20}}, color = {0, 0, 127}));
  end Sensors;

  model Controller
    Modelica.Blocks.Interfaces.RealInput x annotation(
      Placement(visible = true, transformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput y annotation(
      Placement(visible = true, transformation(origin = {-100, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput z annotation(
      Placement(visible = true, transformation(origin = {-100, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput a annotation(
      Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput b annotation(
      Placement(visible = true, transformation(origin = {-100, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput c annotation(
      Placement(visible = true, transformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
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
    Modelica.Blocks.Sources.Constant des_hight(k = 0.0) annotation(
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
  Modelica.Blocks.Continuous.LimPID limPID(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 1, wd = 0.5, wp = 0.001, yMax = 3) annotation(
      Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID limPID1(controllerType = Modelica.Blocks.Types.SimpleController.PD, k = 1, wd = 0.5, wp = 0.001, yMax = 3) annotation(
      Placement(visible = true, transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 0.0) annotation(
      Placement(visible = true, transformation(origin = {-44, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant3(k = 0) annotation(
      Placement(visible = true, transformation(origin = {14, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant4(k = 0.1) annotation(
      Placement(visible = true, transformation(origin = {-28, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
    connect(y, pid.u_m) annotation(
      Line(points = {{-100, 50}, {-100, 49}, {-74, 49}, {-74, 74}, {0, 74}, {0, 78}}, color = {0, 0, 127}));
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
    connect(b, RollPID.u_m) annotation(
      Line(points = {{-100, -50}, {-2, -50}, {-2, -36}, {40.25, -36}, {40.25, -32}, {40, -32}}, color = {0, 0, 127}));
    connect(c, PitchPID.u_m) annotation(
      Line(points = {{-100, -80}, {-2, -80}, {-2, -66}, {37.5, -66}, {37.5, -62}, {38, -62}}, color = {0, 0, 127}));
    connect(a, YawPID1.u_m) annotation(
      Line(points = {{-100, -20}, {-78, -20}, {-78, -96}, {37.5, -96}, {37.5, -92}, {38, -92}}, color = {0, 0, 127}));
    connect(x, positionDecoder.X_abs) annotation(
      Line(points = {{-100, 80}, {-70, 80}, {-70, 30}, {-54, 30}}, color = {0, 0, 127}));
    connect(z, positionDecoder.Y_abs) annotation(
      Line(points = {{-100, 20}, {-70, 20}, {-70, 26}, {-54, 26}}, color = {0, 0, 127}));
    connect(constant1.y, positionDecoder.X_des) annotation(
      Line(points = {{-69, 0}, {-64, 0}, {-64, 20}, {-54, 20}}, color = {0, 0, 127}));
    connect(constant1.y, positionDecoder.Y_des) annotation(
      Line(points = {{-69, 0}, {-64, 0}, {-64, 16}, {-54, 16}}, color = {0, 0, 127}));
    connect(b, positionDecoder.Theta) annotation(
      Line(points = {{-100, -50}, {-44, -50}, {-44, 14}}, color = {0, 0, 127}));
    connect(constant2.y, limPID.u_s) annotation(
      Line(points = {{-32, 50}, {-22, 50}}, color = {0, 0, 127}));
    connect(constant2.y, limPID1.u_s) annotation(
      Line(points = {{-32, 50}, {-26, 50}, {-26, 20}, {-22, 20}}, color = {0, 0, 127}));
    connect(positionDecoder.X_err, limPID.u_m) annotation(
      Line(points = {{-34, 24}, {-30, 24}, {-30, 38}, {-10, 38}}, color = {0, 0, 127}));
    connect(positionDecoder.Y_err, limPID1.u_m) annotation(
      Line(points = {{-34, 20}, {-30, 20}, {-30, 8}, {-10, 8}}, color = {0, 0, 127}));
  connect(PitchPID.y, motorMixingAlgorithm.P) annotation(
      Line(points = {{50, -50}, {56, -50}, {56, -2}, {64, -2}}, color = {0, 0, 127}));
  connect(YawPID1.y, motorMixingAlgorithm.Y) annotation(
      Line(points = {{50, -80}, {60, -80}, {60, -8}, {64, -8}}, color = {0, 0, 127}));
  connect(constant3.y, PitchPID.u_s) annotation(
      Line(points = {{25, 48}, {26, 48}, {26, -50}}, color = {0, 0, 127}));
  connect(constant4.y, YawPID1.u_s) annotation(
      Line(points = {{-16, -90}, {26, -90}, {26, -80}}, color = {0, 0, 127}));
    annotation(
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"),
      Diagram);
  end Controller;

  model Wind
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(table = [0, 0, 0, 0; 10, 0, 0, 0]) annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque torque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
      Placement(visible = true, transformation(origin = {-50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable1(table = [0, 0, 0, 0; 3, 0, 0, 0; 5, 0.001, 0, 0; 6, 0, 0, 0; 8, 0, 0, 0; 9, 0, 0, 0; 10, 0, 0, 0]) annotation(
      Placement(visible = true, transformation(origin = {0, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(worldForce2.frame_b, frame_b) annotation(
      Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(combiTimeTable.y, worldForce2.force) annotation(
      Line(points = {{-10, 0}, {-38, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(torque.frame_b, frame_b) annotation(
      Line(points = {{-60, -30}, {-80, -30}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(torque.torque, combiTimeTable1.y) annotation(
      Line(points = {{-38, -30}, {-10, -30}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Documentation,
      __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl", variableFilter = ".*"),
      Diagram(graphics = {Text(origin = {37, -29}, extent = {{-17, 13}, {17, -13}}, textString = "[x, y,z]"), Text(origin = {56, 2}, extent = {{-32, 6}, {32, -6}}, textString = "[0, 0, 0, 0; 3, 0, 0.01, 0; 5, 0, 0.015, 0; 6, 0, 0.08, 0; 8, 0, 0.0, 0; 9, 0, -0.005, 0; 10, 0, 0, 0]")}));
  end Wind;
  annotation(
    uses(Modelica(version = "4.0.0"), Modelica_DeviceDrivers(version = "2.1.1")));
end QC;
