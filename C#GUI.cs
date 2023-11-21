using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Reflection.Emit;
using System.Runtime.ExceptionServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Button;

namespace Lab1Exercise7
{
    public partial class Form1 : Form
    {
        string serialDataString = "";
        ConcurrentQueue<byte> dataQueue = new ConcurrentQueue<byte>();
        ConcurrentQueue<Int32> acceleration = new ConcurrentQueue<Int32>();
        int datastate = 0;
        StreamWriter outputFile;
        int stepsPerRev = 50;
        double radius = 0.75;
        double timeStep = 1/15/4;
        int position = 0;
        double velocity = 0;

        private Stopwatch stopwatch;
        private List<Point> positionDataPoints;
        private List<Point> velocityDataPoints;
        private List<DataPoint> velocityData = new List<DataPoint>();
        private int timeRange = 1000; // Time range in milliseconds
        private object positionLock = new object();




        public Form1()
        {
            InitializeComponent();
            InitializeChart();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            serialPort1.PortName = "COM9";
            serialPort1.Dispose();
            serialPort1.Open();
            
         
            Timer MyTimer = new Timer();
            MyTimer.Interval = 10;
            MyTimer.Tick += new EventHandler(MyTimer_Tick);
            MyTimer.Start();
            textBox1.Text = "255";
            textBox2.Text = "127";
            textBox3.Text = "0";
            textBox4.Text = "0";
            textBox6.Text = "2";
            trackBar1.ValueChanged += trackBar1_ValueChanged;
            trackBar2.ValueChanged += trackBar2_ValueChanged;
       



        }
        private void MyTimer_Tick(object sender, EventArgs e)
        {
            byte temp;
            byte first =0;
            byte second =0;
            int result;
            byte control;
            int prevPos;
            int mul = 1;

            lock (positionLock)
            {
                while (dataQueue.Count >= 5)
                {
                    dataQueue.TryDequeue(out temp); //startByte
                    if (temp == 255)
                    {
                        dataQueue.TryDequeue(out control); // controlByte
                        dataQueue.TryDequeue(out first); //DB1
                        dataQueue.TryDequeue(out second); //DB2
                        dataQueue.TryDequeue(out temp); //EB
                        if (temp == 0b1)
                        {
                            second = 255;
                        }
                        if (temp == 0b10)
                        {
                            first = 255;
                        }
                        if (temp == 0b11)
                        {
                            first = 255;
                            second = 255;
                        }

                        if (control == 0)
                        {
                            mul = 1;
                        }
                        else
                        {
                            mul = -1;
                        }

                        
                        result = second;
                        result += first << 8;
                        

                        prevPos = position;
                        position = mul * (int)result;
                        textBox9.Text = position.ToString();
                        velocity = (position - prevPos) / (0.01);
                        textBox10.Text = velocity.ToString();
                        UpdateChart(position, velocity);

                    }
       
                    
                }
            }
                



        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
              
            byte newByte = 0;
            int bytesToRead;
            bytesToRead = serialPort1.BytesToRead;
            
            
            
            while (bytesToRead != 0)
            {
                newByte = (byte)serialPort1.ReadByte();
                dataQueue.Enqueue(newByte);
                bytesToRead = serialPort1.BytesToRead;
            }
            
            //
            /*BeginInvoke(new Action(() =>
            {
                textBox9.Text = position.ToString();
               // textBox10.Text = velocity.ToString();
            }));*/

        }
       


        private void button1_Click(object sender, EventArgs e)
        {
            
            byte[] data = { Convert.ToByte(textBox1.Text), Convert.ToByte(textBox6.Text), 
                Convert.ToByte(textBox2.Text), Convert.ToByte(textBox3.Text), Convert.ToByte(textBox4.Text) };
             serialPort1.Write(data, 0, data.Length);

        }



        private void trackBar1_ValueChanged(object sender, EventArgs e)
        {
            int dir = 2;
            ushort newValue;
            if (trackBar1.Value >= 0)
            {
                dir = 2;
                newValue = (ushort)trackBar1.Value;
            }

            else
            {
                newValue = (ushort)(Math.Abs(trackBar1.Value));
                dir = 3;
                
            }
               
            
            

            byte first = (byte)(newValue >> 8);
            byte second = (byte)(newValue);



            byte[] data = { Convert.ToByte(textBox1.Text), Convert.ToByte(dir),
                Convert.ToByte(first), Convert.ToByte(second), Convert.ToByte(textBox4.Text) };
            serialPort1.Write(data, 0, data.Length);

            textBox7.Text = Convert.ToString(newValue);

        }

        private void trackBar2_ValueChanged(object sender, EventArgs e)
        {
            int dir = 4;
            if (trackBar2.Value >= 0)
            {
                dir = 4;
            }

            else
                dir = 5;
            ushort newValue = (ushort)(65535-Math.Abs(trackBar2.Value)+1);
            byte first = (byte)(newValue >> 8);
            byte second = (byte)(newValue);

            byte[] data = { Convert.ToByte(textBox1.Text), Convert.ToByte(dir),
                Convert.ToByte(first), Convert.ToByte(second), Convert.ToByte(textBox4.Text) };
             serialPort1.Write(data, 0, data.Length);

            textBox8.Text = Convert.ToString((double)(65535-newValue+1)/65535*100);
            //textBox8.Text = Convert.ToString(65535 - newValue + 1);
            textBox2.Text = Convert.ToString(first);
            textBox3.Text = Convert.ToString(second);
        }

        private void InitializeChart()
        {
            // Customize your chart settings here
            chart1.Series.Clear();
            chart1.Series.Add("PositionData");
            chart1.Series["PositionData"].ChartType = SeriesChartType.Line;
            chart1.Series["PositionData"].Points.AddY(0); // Add an initial data point

            chart2.Series.Clear();
            chart2.Series.Add("VelocityData");
            chart2.Series["VelocityData"].ChartType = SeriesChartType.Line;
            chart2.Series["VelocityData"].Points.AddY(0); // Add an initial data point
        }
        private void UpdateChart(double positionValue, double velocityValue)
        {
            UpdatePositionChart(positionValue);
            UpdateVelocityChart(velocityValue);
        }

        private void UpdatePositionChart(double positionValue)
        {
            if (chart1.InvokeRequired)
            {
                chart1.BeginInvoke(new Action(() => UpdatePositionChart(positionValue)));
            }
            else
            {
                // Add a new data point to the position chart
                chart1.Series["PositionData"].Points.AddY(positionValue);

                // Limit the number of data points on the X-axis
                int maxDataPoints = 1000;
                while (chart1.Series["PositionData"].Points.Count > maxDataPoints)
                {
                    chart1.Series["PositionData"].Points.RemoveAt(0);
                }

                // Always show the latest data within a fixed range on the X-axis
                double currentX = chart1.Series["PositionData"].Points.Count;
                chart1.ChartAreas[0].AxisX.Minimum = Math.Max(0, currentX - maxDataPoints);
                chart1.ChartAreas[0].AxisX.Maximum = currentX;

                // Auto-adjust the Y-axis range
                chart1.ResetAutoValues();

                // Refresh the chart to display the updated data and scale
                chart1.Update();
            }
        }

        private void UpdateVelocityChart(double velocityValue)
        {
            if (chart2.InvokeRequired)
            {
                chart2.BeginInvoke(new Action(() => UpdateVelocityChart(velocityValue)));
            }
            else
            {
                // Add a new data point to the velocity chart
                chart2.Series["VelocityData"].Points.AddY(velocityValue);

                // Add the data point to the list with the current X value
                velocityData.Add(new DataPoint(chart2.Series["VelocityData"].Points.Count - 1, velocityValue));

                // Always show the latest data within a fixed range on the X-axis
                double currentX = chart2.Series["VelocityData"].Points.Count;
                int maxDataPoints = 40;
                chart2.ChartAreas[0].AxisX.Minimum = Math.Max(0, currentX - maxDataPoints);
                chart2.ChartAreas[0].AxisX.Maximum = currentX;

                // Auto-adjust the Y-axis range
                chart2.ResetAutoValues();

                // Refresh the chart to display the updated data and scale
                chart2.Update();
            }
        }


        private void SavePositionDataToCSV(string filePath)
        {
            var data = chart1.Series["PositionData"].Points.Select(p => new { X = p.XValue, Y = p.YValues[0] });

            var csv = new StringBuilder();
            csv.AppendLine("X,Y");

            foreach (var point in data)
            {
                csv.AppendLine($"{point.X},{point.Y}");
            }

            File.WriteAllText(filePath, csv.ToString());
        }

        private void button2_Click(object sender, EventArgs e)
        {
            string fileName = "data.csv";
            string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = Path.Combine(desktopPath, fileName);

            SavePositionDataToCSV(filePath);
        }

    }
}
