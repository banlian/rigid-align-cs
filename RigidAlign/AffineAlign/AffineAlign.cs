using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RigidAlign.AffineAlign
{
    public class AffineAlign
    {
        public static double[] Transform(double[] pos, double[,] affine)
        {
            var mat = DenseMatrix.OfArray(new double[,]
            {
                {affine[0, 0], affine[0, 1], affine[0, 3]},
                {affine[1, 0], affine[1, 1], affine[1, 3]}
            });

            return mat.Multiply(DenseVector.OfArray(new[] { pos[0], pos[1], 1 })).ToArray();
        }


        public static double[,] Align(double[] sx, double[] sy, double[] tx, double[] ty)
        {
            List<Vector<double>> src = new List<Vector<double>>();
            for (int i = 0; i < sx.Length; i++)
            {
                src.Add(DenseVector.OfArray(new[] { sx[i], sy[i], 1 }));
            }

            var a = DenseMatrix.OfRowVectors(src);

            var vec1 = a.TransposeThisAndMultiply(a).Inverse().Multiply(a.Transpose()).Multiply(DenseVector.OfArray(tx));
            var vec2 = a.TransposeThisAndMultiply(a).Inverse().Multiply(a.Transpose()).Multiply(DenseVector.OfArray(ty));
            var affine = DenseMatrix.OfRowVectors(vec1, vec2);


            //calculate errors
            List<double> errors = new List<double>();
            for (int i = 0; i < src.Count; i++)
            {
                errors.Add((affine * src[i] - DenseVector.OfArray(new[] { tx[i], ty[i] })).PointwisePower(2).Norm(1));
            }

            return new double[,]
            {
                {affine[0, 0], affine[0, 1], 0, affine[0, 2]},
                {affine[1, 0], affine[1, 1], 0, affine[1, 2]},
                {0, 0, 1, 0},
                {0, 0, 0, 1},
            };
        }
    }
}