using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Geocruiser.Behaviors
{
    public class GoToGlobal : Behavior
    {
        private Matrix votes = new Matrix(8, 128);

        private double m_targLat;
        private double m_targLon;

        public override string Name
        {
            get { return "GoToGlobal"; }
        }
        public override Matrix VoteMatrix
        {
            get { return votes; }
        }

        public double TargetLat
        {
            get { return m_targLat; }
            set { m_targLat = value; }
        }

        public double TargetLon
        {
            get { return m_targLon; }
            set { m_targLon = value; }
        }

        public GoToGlobal(Arbiter arb): base(arb)
        {
        }

        public GoToGlobal(double lat, double lon, Arbiter arb) : base(arb)
        {
            m_targLat = lat;
            m_targLon = lon;
        }

        public override Matrix GetVote()
        {
            return GetVote(m_targLat, m_targLon);
        }

        public Matrix GetVote( double target_lat, double target_lon)
        {
            if (target_lat == 0)
                target_lat = arbiter.MC.SV.Position.Est_Global_Lat;

            if (target_lon == 0)
                target_lon = arbiter.MC.SV.Position.Est_Global_Long;

            double boat_x, boat_y, boat_yaw;

            boat_x = arbiter.MC.SV.Position.Est_Global_Lat * Helpers.Lat2Meters;
            boat_y = arbiter.MC.SV.Position.Est_Global_Long * Helpers.Lon2Meters;
            boat_yaw = arbiter.MC.SV.Position.Yaw;

            //put these in meters
            target_lat *= Helpers.Lat2Meters;
            target_lon *= Helpers.Lon2Meters;

            double[] T = { 0, 0 };
            double[] P = { 0, 0 };

            //Target points in boat coordinate frame
            //transform target point to boat coordinates
            T[0] = (target_lat - boat_x) * Math.Cos(boat_yaw) - (target_lon - boat_y) * Math.Sin(boat_yaw);
            T[1] = -((target_lat - boat_x) * Math.Sin(boat_yaw) + (target_lon - boat_y) * Math.Cos(boat_yaw));

            //fill vote matrix according to equation: Vi=1/(norm(Pi-T)+1).  Vi, is vote i, Pi is the location of
            //cell i, and T is the target location.

            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 128; j++)
                {
                    //Calculate cell location in rectangular coordinates
                    P[0] = Radius(i) * Math.Cos(Angle(j));
                    P[1] = Radius(i) * Math.Sin(Angle(j));

                    //normalize by dividing by magnitue
                    //For a 2D vector, magnitude is sqrt(x*x + y*y)
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

            return votes;
        }
    }
}
