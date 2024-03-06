package QC
  model Quadcopter
  QC.Chasis chasis annotation(
      Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  QC.RPU_CounterClockWise rPU_CounterClockWise annotation(
      Placement(visible = true, transformation(origin = {34, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  QC.RPU_CounterClockWise rPU_CounterClockWise1 annotation(
      Placement(visible = true, transformation(origin = {34, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {72, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  QC.RPU_ClockWise rPU_ClockWise annotation(
      Placement(visible = true, transformation(origin = {34, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  QC.RPU_ClockWise rPU_ClockWise1 annotation(
      Placement(visible = true, transformation(origin = {34, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  QC.Sensors sensors annotation(
      Placement(visible = true, transformation(origin = {34, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  QC.Controller controller annotation(
      Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(chasis.frame_a4, sensors.frame_a) annotation(
      Line(points = {{80, 0}, {90, 0}, {90, -88}, {44, -88}}, color = {95, 95, 95}));
  connect(controller.RF, rPU_ClockWise.u) annotation(
      Line(points = {{-10, 8}, {0, 8}, {0, 56}, {24, 56}}, color = {0, 0, 127}));
  connect(controller.RB, rPU_CounterClockWise.u) annotation(
      Line(points = {{-10, 2}, {10, 2}, {10, 20}, {24, 20}}, color = {0, 0, 127}));
  connect(controller.LB, rPU_ClockWise1.u) annotation(
      Line(points = {{-10, -2}, {10, -2}, {10, -24}, {24, -24}}, color = {0, 0, 127}));
  connect(controller.LF, rPU_CounterClockWise1.u) annotation(
      Line(points = {{-10, -6}, {0, -6}, {0, -64}, {24, -64}}, color = {0, 0, 127}));
  connect(sensors.x, controller.x) annotation(
      Line(points = {{24, -80}, {-70, -80}, {-70, 10}, {-30, 10}}, color = {0, 0, 127}));
  connect(sensors.y, controller.y) annotation(
      Line(points = {{24, -82}, {-72, -82}, {-72, 6}, {-30, 6}}, color = {0, 0, 127}));
  connect(sensors.z, controller.z) annotation(
      Line(points = {{24, -84}, {-74, -84}, {-74, 2}, {-30, 2}}, color = {0, 0, 127}));
  connect(sensors.a, controller.a) annotation(
      Line(points = {{24, -90}, {-80, -90}, {-80, -2}, {-30, -2}}, color = {0, 0, 127}));
  connect(sensors.b, controller.b) annotation(
      Line(points = {{24, -92}, {-82, -92}, {-82, -6}, {-30, -6}}, color = {0, 0, 127}));
  connect(sensors.c, controller.c) annotation(
      Line(points = {{24, -94}, {-84, -94}, {-84, -10}, {-30, -10}}, color = {0, 0, 127}));
  connect(rPU_ClockWise.frame_b, chasis.frame_RF) annotation(
      Line(points = {{44, 56}, {54, 56}, {54, 8}, {60, 8}}, color = {95, 95, 95}));
  connect(rPU_CounterClockWise.frame_b, chasis.frame_a_RB) annotation(
      Line(points = {{44, 20}, {48, 20}, {48, 2}, {60, 2}}, color = {95, 95, 95}));
  connect(rPU_ClockWise1.frame_b, chasis.frame_a_LB) annotation(
      Line(points = {{44, -24}, {48, -24}, {48, -2}, {60, -2}}, color = {95, 95, 95}));
  connect(rPU_CounterClockWise1.frame_b, chasis.frame_a_LF) annotation(
      Line(points = {{44, -64}, {54, -64}, {54, -8}, {60, -8}}));
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
  
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 0.05, sphereDiameter = 0.05)  annotation(
      Placement(visible = true, transformation(origin = {4, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldTorque torque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
      Placement(visible = true, transformation(origin = {70, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
      Placement(visible = true, transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain matrixGain(K = [1, 0, 0; 0, 1, 0; 0, 0, 1])  annotation(
      Placement(visible = true, transformation(origin = {38, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-104, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 100, uMin = 0)  annotation(
      Placement(visible = true, transformation(origin = {-52, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {-34, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = 0.01)  annotation(
      Placement(visible = true, transformation(origin = {-22, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(
      Placement(visible = true, transformation(origin = {-34, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 2)  annotation(
      Placement(visible = true, transformation(origin = {-90, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const2(k = 0.1)  annotation(
      Placement(visible = true, transformation(origin = {-90, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-34, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Routing.Multiplex mux(n = 3)  annotation(
      Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Routing.Multiplex mux1(n = 3)  annotation(
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
  Modelica.Blocks.Routing.DeMultiplex demux(n = 3)  annotation(
      Placement(visible = true, transformation(origin = {-40, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Routing.DeMultiplex demux1(n = 3)  annotation(
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
  Modelica.Blocks.Math.Add3 add31(k1 = -1, k3 = -1)  annotation(
        Placement(visible = true, transformation(origin = {30, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add32(k2 = -1, k3 = -1)  annotation(
        Placement(visible = true, transformation(origin = {30, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add33(k1 = -1, k2 = -1)  annotation(
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

    Controller.MotorMixingAlgorithm motorMixingAlgorithm annotation(
      Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 66.2)  annotation(
      Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0)  annotation(
      Placement(visible = true, transformation(origin = {24, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const2(k = 0)  annotation(
      Placement(visible = true, transformation(origin = {24, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const3(k = 0)  annotation(
      Placement(visible = true, transformation(origin = {24, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID pid(Td = 1.5, Ti = 1000,controllerType = Modelica.Blocks.Types.SimpleController.PID, initType = Modelica.Blocks.Types.Init.NoInit, k = 7, yMax = 10, yMin = -10, y_start = 0)  annotation(
      Placement(visible = true, transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant des_hight(k = 0.4)  annotation(
      Placement(visible = true, transformation(origin = {-60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
      Placement(visible = true, transformation(origin = {20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(motorMixingAlgorithm.RF, RF) annotation(
      Line(points = {{70, 6}, {74, 6}, {74, 60}, {100, 60}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.RB, RB) annotation(
      Line(points = {{70, 2}, {80, 2}, {80, 20}, {100, 20}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.LB, LB) annotation(
      Line(points = {{70, -2}, {80, -2}, {80, -20}, {100, -20}}, color = {0, 0, 127}));
    connect(motorMixingAlgorithm.LF, LF) annotation(
      Line(points = {{70, -6}, {74, -6}, {74, -60}, {100, -60}}, color = {0, 0, 127}));
    connect(const1.y, motorMixingAlgorithm.R) annotation(
      Line(points = {{35, 16}, {40, 16}, {40, 2}, {50, 2}}, color = {0, 0, 127}));
    connect(const2.y, motorMixingAlgorithm.P) annotation(
      Line(points = {{35, -16}, {40, -16}, {40, -2}, {50, -2}}, color = {0, 0, 127}));
    connect(const3.y, motorMixingAlgorithm.Y) annotation(
      Line(points = {{35, -46}, {44, -46}, {44, -8}, {50, -8}}, color = {0, 0, 127}));
  connect(des_hight.y, pid.u_s) annotation(
      Line(points = {{-48, 80}, {-42, 80}}, color = {0, 0, 127}));
  connect(y, pid.u_m) annotation(
      Line(points = {{-100, 50}, {-30, 50}, {-30, 68}}, color = {0, 0, 127}));
  connect(const.y, add.u2) annotation(
      Line(points = {{2, 50}, {4, 50}, {4, 54}, {8, 54}}, color = {0, 0, 127}));
  connect(pid.y, add.u1) annotation(
      Line(points = {{-18, 80}, {-4, 80}, {-4, 66}, {8, 66}}, color = {0, 0, 127}));
  connect(add.y, motorMixingAlgorithm.T) annotation(
      Line(points = {{32, 60}, {44, 60}, {44, 8}, {50, 8}}, color = {0, 0, 127}));
  end Controller;
  annotation(
    uses(Modelica(version = "4.0.0"), Modelica_DeviceDrivers(version = "2.1.1")));
end QC;
