using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Geocruiser.Behaviors
{
    public class FollowHeading : Behavior
    {
        private Matrix votes = new Matrix(8, 128);
        private double m_desHeading = 0;

        public override string Name
        {
            get { return "FollowHeading"; }
        }

        public override Matrix VoteMatrix
        {
            get { return votes; }
        }

        public double DesiredHeading
        {
            get { return m_desHeading; }
            set { m_desHeading = value; }
        }

        public FollowHeading(Arbiter arb): this(0, arb)
        {
        }

        public FollowHeading(double desiredHeading, Arbiter arb) : base(arb)
        {
            m_desHeading = desiredHeading;
        }

        public override Matrix GetVote()
        {
            return GetVote(m_desHeading);
        }

        public Matrix GetVote( double desiredHeading )
        {
            double yaw = arbiter.MC.SV.Position.Yaw;

            if (yaw < 0)
                yaw += 2 * Math.PI;
            while (desiredHeading < 0)
                desiredHeading += 2 * Math.PI;
            while (desiredHeading > 2 * Math.PI)
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