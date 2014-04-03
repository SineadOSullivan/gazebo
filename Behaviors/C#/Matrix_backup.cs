// Assignment Classes
using System;
using System.Collections;
using System.Collections.Generic;

namespace Geocruiser.Behaviors
{
    /// <summary>
    /// A Matrix Class
    /// <summary>
    public class Matrix
    {
        private string m_name;
        private double[,] data = null;

        public int Width
        {
            get { return data.GetLength(0); }
        }
        public int Height
        {
            get { return data.GetLength(1); }
        }
        public string Name
        {
            get { return m_name; }
            set { m_name = value; }
        }

        public Matrix()
            : this(1, 1, string.Empty)
        {
        }

        public Matrix(int n, int m)
            : this(n, m, string.Empty)
        {
        }

        public Matrix(int n, int m, string name)
        {
            data = new double[n, m];
            m_name = name;
        }

        public double this[int n, int m]
        {
            get
            {
                return data[n, m];
            }
            set
            {
                data[n, m] = value;
            }
        }

        /// <summary>
        /// Loads the identity matrix into this one.
        /// </summary>
        public void loadIdentity()
        {
            for (int j = 0; j < Height; j++)
                for (int i = 0; i < Width; i++)
                    if (i == j)
                        data[i, j] = 1;
                    else
                        data[i, j] = 0;
        }

        /// <summary>
        /// Sets this matrix to the data of another.
        /// </summary>
        /// <param name="other"></param>
        public void set(Matrix other)
        {
            for (int j = 0; j < Height; j++)
                for (int i = 0; i < Width; i++)
                    data[i, j] = other[i, j];
        }

        public double[] row(int n)
        {
            if (Width < n)
                throw new Exception("Out-of-bounds of matrix: specified row is too high");
            else
            {
                double[] rowdata = new double[Height];
                for (int i = 0; i < Height; i++)
                    rowdata[i] = data[n, i];
                return rowdata;
            }
        }

        public bool matrixValid()
        {
            double max = MaxCoeff();
            double min = MinCoeff();

            if (max > 100000 || min < -100000)
                return false;
            else
                return true;
        }

        #region operators
        public static Matrix operator +(Matrix m1, Matrix m2)
        {
            //check to make sure they are same size
            if (m1.Height != m2.Height || m1.Width != m2.Width)
                return null;
            else
            {
                Matrix n = new Matrix(m1.Width, m1.Height);
                for (int j = 0; j < m1.Height; j++)
                    for (int i = 0; i < m1.Width; i++)
                        n[i, j] = m1[i, j] + m2[i, j];
                return n;
            }
        }

        public static Matrix operator *(Matrix m1, double d)
        {
            for (int j = 0; j < m1.Height; j++)
                for (int i = 0; i < m1.Width; i++)
                    m1[i, j] *= d;
            return m1;
        }

        public static Matrix operator /(Matrix m1, double d)
        {
            if( d == 0 )
                return m1;

            for (int j = 0; j < m1.Height; j++)
                for (int i = 0; i < m1.Width; i++)
                    m1[i, j] /= d;
            return m1;
        }
        #endregion
        #region coefficients
        public double MaxCoeff()
        {
            int row = 0, col = 0;
            return MaxCoeff(ref row, ref col);
        }

        public double MaxCoeff(ref int row, ref int col)
        {
            int tempr = -1;
            int tempc = -1;
            double tempMax = int.MinValue;

            for (int j = 0; j < Height; j++)
            {
                for (int i = 0; i < Width; i++)
                {
                    if (data[i, j] > tempMax)
                    {
                        tempMax = data[i, j];
                        tempr = i;
                        tempc = j;
                    }
                }
            }
            row = tempr;
            col = tempc;
            return tempMax;
        }
        public double MinCoeff()
        {
            int row = 0, col = 0;
            return MinCoeff(ref row, ref col);
        }

        public double MinCoeff(ref int row, ref int col)
        {
            int tempr = -1;
            int tempc = -1;
            double tempMin = int.MaxValue;

            for (int j = 0; j < Height; j++)
            {
                for (int i = 0; i < Width; i++)
                {
                    if (data[i, j] < tempMin)
                    {
                        tempMin = data[i, j];
                        tempr = i;
                        tempc = j;
                    }
                }
            }
            row = tempr;
            col = tempc;
            return tempMin;
        }
#endregion
    }
}