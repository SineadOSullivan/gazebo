using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Geocruiser.Behaviors
{
    class LaserAvoid : Behavior
    {
        private Matrix votes = new Matrix(8, 128);
        private double l = 0.75;
        private double w = 0.75;

        public override string Name
        {
            get { return "LaserAvoid"; }
        }

        public override Matrix VoteMatrix
        {
            get { lock (this) { return votes; } }
        }

        public LaserAvoid(Arbiter arb): base(arb)
        {
        }

        public override Matrix GetVote()
        {
            //if (Math.Abs(State_Vehicle.Instance.Position.Pitch) > 0.26 || MissionCommander.Instance.LidarData == null ) 
            if (arbiter.MC == null || arbiter.MC.LidarData == null)
                return votes;

            lock (this)
            {
                double[] laserdata = arbiter.MC.LidarData;
                double[] i2r = Behaviors.Behavior.index2radius;
                votes = new Matrix(8, 128);

                double range, bearing, r1, r2, t1, t2, DW, sr, RA, ang, gamma, alpha, rd, C;
                int index;
                int numangs = 128;

                //Radians per sample
                ang = 3.14159 * 2 / numangs;

                //Fill matrix for each laser point
                for (int i = 0; i < 1080; i++)
                {
                    //Laser data is already divided by 1000 to convert from mm to m
                    range = laserdata[i];
                    if (range < 0.15)
                        range = 20;
                    bearing = -3 * Math.PI / 4 + i * 3 * Math.PI / (2 * 1080);
                    for (int j = 0; j < 2; j++)
                    {
                        if (range <= w || range <= l)
                        {
                            r1 = 0;
                            r2 = 0;
                            t1 = bearing;
                            t2 = bearing;
                        }
                        else if (j == 0)
                        {
                            r1 = Math.Sqrt((range * range) - (w * w));
                            r2 = range - l;
                            t1 = bearing - Math.Asin(w / range);
                            t2 = bearing;
                        }
                        else
                        {
                            r1 = range - l;
                            r2 = Math.Sqrt((range * range) - (w * w));
                            t1 = bearing;
                            t2 = bearing + Math.Asin(w / range);
                        }
                        DW = t2 - t1;
                        sr = r2 * Math.Sin(DW) / w;
                        RA = Math.Asin(sr);
                        for (int k = (int)Math.Round(t1 / ang); k <= Math.Round(t2 / ang); k++)
                        {
                            gamma = k * ang;
                            C = gamma - t1;
                            alpha = 3.14159 - C - RA;
                            rd = r1 * sr / Math.Sin(alpha);

                            if (k < 0)
                                index = k + numangs;
                            else
                                index = k;

                            for (int loop = 0; loop < i2r.Length; loop++)
                                if (rd < i2r[loop])
                                    votes[loop, index] = -1;

                            /*M(index,0)=rd;
                            M(index,1)=r1;
                            M(index,2)=t1*180/3.14;
                            M(index,3)=r2;
                            M(index,4)=t2*180/3.14;
                            M(index,5)=k*ang*180/3.14;*/
                        }
                    }
                }
                /*for (int i = (int)Math.Round(3.14 / (2 * ang)); i <= (3 * 3.14 / (2 * ang)); i++)
                {
                    //M(i,2)=-1;
                    //M(i,3)=-1;
                    //votes(4,i)=-1;
                    //votes(5,i)=-1;
                }*/
                return votes;
            }
        }

    }
}
