package com.hansRobotBaseLib;

public class VariableType {
    public class StringData
    {
        String s_data;
        public void setReturn(String data) {
            this.s_data = data;
        }
        public String getReturn() {
            return s_data;
        }
        public String toString() {
            return String.format("%s", s_data);
        }
    }

    public class IntData
    {
        int IntData;
        public void setReturn(int data) {
            this.IntData = data;
        }
        public int getReturn() {
            return IntData;
        }
        public String toString() {
            return String.format("%d", IntData);
        }
    }

    public class DoubleData
    {
        double DoubleData;
        public void setReturn(double data) {
            this.DoubleData = data;
        }
        public double getReturn() {
            return DoubleData;
        }
        public String toString() {
            return String.format("%f", DoubleData);
        }
    }

    public class BooleanData
    {
        boolean BooleanData;
        public void setReturn(boolean data) {
            this.BooleanData = data;
        }
        public Boolean getReturn() {
            return BooleanData;
        }
        public String toString() {
            return String.format("%s", BooleanData);
        }
    }

    public class JointsData
    {
        public DoubleData[] joint = new DoubleData[15];

    }

    public class JointsDatas
    {
        public double[] joint = new double[15];

    }

    public class Position
    {
        public double dx;
        public double dy;
        public double dz;
        public double dRx;
        public double dRy;
        public double dRz;
    }

    public class JointPosition
    {
        public double dJ1;
        public double dJ2;
        public double dJ3;
        public double dJ4;
        public double dJ5;
        public double dJ6;
    }
}
