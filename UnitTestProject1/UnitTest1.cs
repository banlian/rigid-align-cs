using System;
using System.Collections.Generic;
using HalconDotNet;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Linq;

namespace UnitTestProject1
{

    struct PosXY
    {
        public double X;
        public double Y;
    }



    [TestClass]
    public class UnitTest1
    {
        [TestMethod]
        public void TestMethod1()
        {
            //220.2   1531.51 - 226.423    340.238
            //951.28  2967.35 - 303.982    335.088
            //2386.24 2233.19 - 308.787    413.195
            //1653.1  800.28 - 231.388    417.857

            List<PosXY> campos = new List<PosXY>()
                                 {
                                     new PosXY() {X = 220.2, Y = 1531.51},
                                     new PosXY() {X = 951.28, Y = 2967.35},
                                     new PosXY() {X = 2386.24, Y = 2233.19},
                                     new PosXY() {X = 1653.1, Y = 800.28},
                                 };

            List<PosXY> robpos = new List<PosXY>()
                                 {
                                     new PosXY() {X = -226.423, Y = 340.238},
                                     new PosXY() {X = -303.982, Y = 335.088},
                                     new PosXY() {X = -308.787, Y = 413.195},
                                     new PosXY() {X = -231.388, Y = 417.857},
                                 };

            Console.WriteLine($"CAMPOS:\r\n{string.Join("\r\n", campos.Select(p => p.ToString()))}\r\n");
            Console.WriteLine($"ROBOTPOS:\r\n{string.Join("\r\n", robpos.Select(p => p.ToString()))}\r\n");

            //X缩放: 0.048
            //Y缩放: 0.048
            //坐标系旋转: 2.104
            //Y轴错切: 0.004
            //X轴平移: -157.528
            //Y轴平移: 368.798



            Console.WriteLine("HALCON CALIBRATION: \r\n");

            HHomMat2D coorCalMat = new HHomMat2D();

            HTuple cx = new HTuple(campos.Select(d => d.X).ToArray());
            HTuple cy = new HTuple(campos.Select(d => d.Y).ToArray());
            HTuple rx = new HTuple(robpos.Select(d => d.X).ToArray());
            HTuple ry = new HTuple(robpos.Select(d => d.Y).ToArray());

            coorCalMat.VectorToHomMat2d(cx, cy, rx, ry);

            //calculate halcon errors
            List<double> errors = new List<double>();
            for (int i = 0; i < campos.Count; i++)
            {
                double rx1, ry1;

                rx1 = coorCalMat.AffineTransPoint2d(campos[i].X, campos[i].Y, out ry1);

                errors.Add((robpos[i].X - rx1) * (robpos[i].X - rx1) + (robpos[i].Y - ry1) * (robpos[i].Y - ry1));
            }
            Console.WriteLine($"HALCON FIT MAX ERROR: {errors.Max():F3}\r\n");


            double sx, sy, phi, theta, tx, ty;
            sx = coorCalMat.HomMat2dToAffinePar(out sy, out phi, out theta, out tx, out ty);
            Console.WriteLine($"sx:{sx:F6}\r\nsy:{sy:F6}\r\nphi:{phi:F6}\r\ntheta:{theta:F6}\r\ntx:{tx:F6}\r\nty:{ty:F6}\r\n");


            {
                double rx1, ry1;

                rx1 = coorCalMat.AffineTransPoint2d(220.2, 1531.51, out ry1);

                Console.WriteLine("\r\nHALCON transform 220.2, 1531.51 into " + rx1.ToString("F6") + "," + ry1.ToString("F6"));
            }


            {
                Console.WriteLine("\r\nRIGID ALIGN CALIBRATION: \r\n");

                var res = RigidAlign.RigidAlign.Align(campos.Select(d => d.X).ToArray(), campos.Select(d => d.Y).ToArray(), robpos.Select(d => d.X).ToArray(), robpos.Select(d => d.Y).ToArray());

                Console.WriteLine("RIGID ALIGN MAX ERROR:" + res.Item2.ToString("F6") + "\r\n");

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (j == 2)
                        {
                            Console.Write($"{res.Item1[i, j]:F6}\r\n");
                        }
                        else
                        {
                            Console.Write($"{res.Item1[i, j]:F6},");
                        }
                    }
                }

                Console.WriteLine("\r\nRIGID ALIGN transform 220.2, 1531.51 into " + string.Join(",", RigidAlign.RigidAlign.Transform(new[] { 220.2, 1531.51 }, res.Item1).Select(d => d.ToString("F3"))));
            }
        }
    }
}