using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;
using System.Threading;

namespace Geocruiser.Behaviors
{
    public class Arbiter
    {

#region Events
        //define what the delegate is
        public delegate void ArbiterVoteEventHandler(object sender, ArbiterEventArgs e);
        //tihs is our instance for the delegate
        public static event ArbiterVoteEventHandler VoteEventHandler;

        /// <summary>
        /// this class is the message that will be passed when the Arbiter Event is raised
        /// </summary>
        public class ArbiterEventArgs : EventArgs
        {
            public double Target_R;
            public double Target_Theta;
            public double Target_X { get { return Target_R * Math.Cos(Target_Theta); } }
            public double Target_Y { get { return Target_R * Math.Sin(Target_Theta); } }

            public ArbiterEventArgs(double target_R, double target_Theta)
            {
                Target_R = target_R;
                Target_Theta = target_Theta;
            }
        }
#endregion

        private MissionCommander m_mc;

        private Dictionary<string, double> weights = new Dictionary<string, double>();
        private Dictionary<string, Behavior> behaviors= new Dictionary<string, Behavior>();
        private Matrix arbMatrix = new Matrix(8, 128);

        private int m_interval = 100;
        private Thread m_thread;
        private volatile bool working = false;

        public MissionCommander MC
        {
            get { return m_mc; }
        }

        public int TimerInterval
        {
            get { return m_interval; }
            set { m_interval = value; }
        }

        public Matrix VoteMatrix
        {
            get { lock (this) { return arbMatrix; } }
        }

        public bool Working
        {
            get
            {
                lock (this)
                {
                    return working;
                }
            }
        }

        public bool Active
        {
            get 
            {
                lock (this)
                {
                    if (m_thread == null)
                        return false;
                    else
                        return m_thread.IsAlive;
                }
            }
        }

        public Arbiter(MissionCommander mc)
        {
            m_mc = mc;
            InitializeBehaviorList();
            m_thread = new Thread(RunVotes);
            m_thread.Name = "Arbiter";
            m_thread.IsBackground = true;
        }

        private void InitializeBehaviorList()
        {
            AddBehavior(new GoToGlobal(this));
            AddBehavior(new GoToLocal(this));
            AddBehavior(new FollowHeading(this));
            AddBehavior(new LaserAvoid(this));
            AddBehavior(new BuoyCentroid(this));
        }

        public void ArbiterStart()
        {
            if ( working || Active)
                return;
            else
            {
                lock (this)
                {
                    working = true;
                }
                m_thread = new Thread(RunVotes);
                m_thread.Name = "Arbiter";
                m_thread.IsBackground = true;
                m_thread.Start();
            }
        }

        public void ArbiterStop()
        {
            lock (this)
            {
                working = false;
                m_thread.Abort();
                while (m_thread.IsAlive)
                    Thread.Sleep(10);
            }
        }

        public void RunVotes()
        {
            while (working)
            {
                GetVote();
                Thread.Sleep(m_interval);
            }
        }

        private Matrix GetVote()
        {
            Matrix sum = new Matrix(8, 128);
            int row = -1;
            int col = -1;
            double r, theta;

            lock (this)
            {
                foreach (KeyValuePair<string, double> kvp in weights)
                {
                    if (kvp.Value != 0 && behaviors.ContainsKey(kvp.Key))
                    {
                        Matrix tm = behaviors[kvp.Key].GetVote() * kvp.Value;
                        sum += tm;
                    }
                }
            }
            sum.MaxCoeff(ref row, ref col);

            r = Behavior.Radius(row);
            theta = Behavior.Angle(col);

            OnUpdatedVote(new ArbiterEventArgs(r, theta));
            Utilities.Log.WriteLine("Target:: X=" + r * Math.Cos(theta) + " Y=" + r * Math.Sin(theta));
            lock (this)
                arbMatrix = sum;
            return sum;
        }

        protected virtual void OnUpdatedVote(ArbiterEventArgs e)
        {
            if (VoteEventHandler != null)
                VoteEventHandler(this, e);
        }

        public void SetGain(string behaviorName, double w)
        {
            lock (this)
            {
                weights[behaviorName] = w;
            }
        }

        public void AddBehavior(Behavior b)
        {
            lock (this)
            {
                behaviors[b.Name] = b;
                weights[b.Name] = 0;
            }
        }

        public void RemoveBehavior(Behavior b)
        {
            lock (this)
            {
                behaviors.Remove(b.Name);
                weights.Remove(b.Name);
            }
        }

        public void ClearGains()
        {
            lock (this)
            {
                Dictionary<string, double> wtemp = new Dictionary<string, double>();
                foreach (KeyValuePair<string, double> kvp in weights)
                    wtemp[kvp.Key] = 0;
                weights = wtemp;
            }
        }

        public string GetBehaviorName(int i)
        {
            if (i < behaviors.Count)
            {
                KeyValuePair<string, Behavior> kvp = behaviors.ElementAt(i);
                return kvp.Key;
            }
            else return string.Empty;
        }

        public Behavior GetBehavior(string name)
        {
            if (behaviors.ContainsKey(name))
                return behaviors[name];
            else
                return null;
        }
    }
}