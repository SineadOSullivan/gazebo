using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Utilities;

namespace Geocruiser.Behaviors
{
    class GoToLocal : Behavior
    {
        private Matrix votes = new Matrix(8, 128);

        private double m_targX = 0;
        private double m_targY = 0;

        public override string Name
        {
            get { return "GoToLocal"; }
        }

        public override Matrix VoteMatrix
        {
            get { return votes; }
        }

        public double TargetX
        {
            get { return m_targX; }
            set { m_targX = value; }
        }

        public double TargetY
        {
            get { return m_targY; }
            set { m_targY = value; }
        }

        public GoToLocal(Arbiter arb) : this(0, 0, arb)
        {
        }

        public GoToLocal(double tx, double ty, Arbiter arb) : base(arb)
        {
            m_targX = tx;
            m_targY = ty;
        }

        public override Matrix GetVote()
        {
            return GetVotes(m_targX, m_targY);
        }

        public Matrix GetVotes(double target_x, double target_y)
        {
            double[] T = { 0, 0 };
            double[] P = { 0, 0 };

            //Target points in boat coordinate frame
            //transform target point to boat coordinates
            T[0] = target_x;
            T[1] = target_y;

            Log.WriteLine("GoToLocal: x: " + target_x + " y: " + target_y);

            //fill vote matrix according to equation: Vi=1/(norm(Pi-T)+1).  Vi, is vote i, Pi is the location of
            //cell i, and T is the target location.

            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 128; j++)
                {
                    //Calculate cell location in rectangular coordinates
                    P[0] = Radius(i) * Math.Cos(Angle(j));
                    P[1] = Radius(i) * Math.Sin(Angle(j));

                    //Calculate vote according to distance between cell and target
                    double[] Temp = { T[0] - P[0], T[1] - P[1] };
                    double mag = Math.Sqrt(Math.Pow(Temp[0], 2) + Math.Pow(Temp[1], 2));
                    VoteMatrix[i, j] = 1 / (mag + 1);
                }
            }

            //Special Case: votes(0,127) corresponds to r=0:
            double tmag = Math.Sqrt(Math.Pow(T[0], 2) + Math.Pow(T[1], 2));
            VoteMatrix[0, 127] = 1 / (tmag + 1);

            //Scale votes so the maximum vote=1
            votes /= VoteMatrix.MaxCoeff();

            return VoteMatrix;
        }
    }
}
