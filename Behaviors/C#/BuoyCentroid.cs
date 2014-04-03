using System;
using System.Collections.Generic;
using System.Text;
using Geocruiser;

namespace Geocruiser.Behaviors
{
    class BuoyCentroid : Behavior
    {
        private Matrix votes = new Matrix(8, 128);
        private double[] m_GPSendpoint = new double[2];
        private double m_lastHeading = 0;

        public override string Name
        {
            get { return "Centroid"; }
        }

        public override Matrix VoteMatrix
        {
            get { return votes; }
        }

        public double[] GPS_Endpoint
        {
            get { return m_GPSendpoint; }
            set { m_GPSendpoint = value; }
        }
        public BuoyCentroid(Arbiter arb) : this( new double[]{0,0}, arb)
        {
        }
        public BuoyCentroid(double[] GPS_End, Arbiter arb) : base(arb)
        {
            m_GPSendpoint = GPS_End;
        }

        public override Matrix GetVote()
        {
            return GetVote();
        }

        public Matrix GetVote(double desiredHeading)
        {
            double xsum = 0;
            double ysum = 0;
            int count = 0;
            for (int i = 0; i < this.arbiter.MC.Buoys.Count; i++)
            {
                Buoy mb = this.arbiter.MC.Buoys[i];
                double[] pos = mb.GetXYCoors(this.arbiter.MC.SV);
                if (pos[0] > 0)//if bouy is in front of us
                {
                    xsum += pos[0];
                    ysum += pos[1];
                    count++;
                }
            }

            double gain = 0.5;
            double dh = m_lastHeading;
            double theta = dh;//just set this here for now

            if (count > 1)
                theta = Math.Atan2(ysum, xsum) + this.arbiter.MC.SV.Position.Yaw;
            else if( m_GPSendpoint[0] != 0 && m_GPSendpoint[1] != 0 )
            {
                double x = double.MaxValue;
                double y = double.MaxValue;
                gain = 0.0045;
                this.arbiter.MC.SV.Global2Boat(m_GPSendpoint[0], m_GPSendpoint[1], ref x, ref y);
                theta = Math.Atan2(y, x) + this.arbiter.MC.SV.Position.Yaw;
            }

            if (theta - dh < -Math.PI)//normalize to between 0 and 2 pi
                theta += 2 * Math.PI;
            if (theta - dh > Math.PI)
                theta -= 2 * Math.PI;

            m_lastHeading = dh * (1 - gain) + gain * theta;
            return FillVote(m_lastHeading);
        }

        /// <summary>
        /// Lifted straight from FollowHeading
        /// </summary>
        /// <param name="desiredHeading"></param>
        /// <returns></returns>
        private  Matrix FillVote( double desiredHeading )
        {
            double yaw = arbiter.MC.SV.Position.Yaw;

            if (yaw < 0)
                yaw += 2 * Math.PI;
            if (desiredHeading < 0)
                desiredHeading += 2 * Math.PI;
            if (desiredHeading > 2 * Math.PI)
                desiredHeading -= 2 * Math.PI;

            //Compute vote matrix: vote higher for directions closer to desired heading
            double shift = Math.Round((desiredHeading - yaw) * 64.0 / Math.PI);
            if (shift < 0)
                shift = shift + 128;

            int i = (int)shift;
            while (i < shift + 65 && i < 128)
            {
                VoteMatrix[7, i] = 1 - (i - shift) / 32;
                i++;
            }

            while (i < 128)
            {
                VoteMatrix[7, i] = 1 - (shift + 128 - i) / 32;
                i++;
            }

            i = (int)(shift - 1);
            while (i > shift - 65 && i >= 0 && i < 128)
            {
                VoteMatrix[7, i] = 1 - (shift - i) / 32;
                i--;
            }

            while (i >= 0 && i < 128)
            {
                VoteMatrix[7, i] = 1 - (i - shift + 128) / 32;
                i--;
            }

            double[] lastRow = VoteMatrix.GetRow(VoteMatrix.RowCount-1);
            for (int j = 0; j < VoteMatrix.RowCount; j++)
                for (int k = 0; k < lastRow.Length; k++)
                    if (k < 128)
                        VoteMatrix[j, k] = lastRow[k] * (j + 1) / 8;

            //Don't forget special case: votes(0,127)
            VoteMatrix[0, 127] = 0;

            return votes;
        }
    }
}