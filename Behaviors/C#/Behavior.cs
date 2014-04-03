using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Geocruiser.Behaviors
{
    public abstract class Behavior
    {
        protected Arbiter arbiter;
        public abstract string Name{ get;}
        public abstract Matrix VoteMatrix { get; }

        public static double[] index2radius = { 0.1, 0.4, 0.8, 1.3, 2.0, 3.0, 5.0, 8.0 };

        // static
        public static double Radius(int index)
        {
            if (index > index2radius.Length)
                throw new Exception("Index greater than 7.");
            else
                return index2radius[index];
        }
        // static
        public static double Angle(int index)
        {
            if (index > 128)
                index = 128;

            return index * Math.PI / 64;
        }

        public Behavior(Arbiter arb)
        {
            arbiter = arb;
        }

        public abstract Matrix GetVote();
    }
}
