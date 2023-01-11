using HRV;
using TTestApp.Commands;
using TTestApp.Decomposers;
using TTestApp.Enums;
using TTestApp.Spline;

namespace TTestApp
{
    public partial class Form1 : Form, IMessageHandler
    {
        USBSerialPort USBPort;
        DataArrays? DataA;
        ByteDecomposer Decomposer;
        WaveDetector? Detector;
        CurvesPainter Painter;
        BufferedPanel BufPanel;
        TTestConfig Cfg;
        StreamWriter TextWriter;
        ConverterValueToMmHg ValueToMmHg;
        string CurrentFile;
        int CurrentFileSize;
        const string TmpDataFile = "tmpdata.t";
        int MaxValue = 200;   // Для ADS1115
        bool ViewMode = false;
        int ViewShift;
        double ScaleY = 1;
        List<int[]> VisirList;
        PressureMeasurementStatus PressureMeasStatus = PressureMeasurementStatus.Ready;
        PumpingStatus PumpStatus = PumpingStatus.Ready;
        double CurrentPressure;
        double MaxAllowablePressure = 180;
        double MinPressure = 120;
        double MomentMaxFound;
        double MaxPressureValue;
        double MaxTimeAfterMaxFound = 6; //sec
        double MaxTimeAfterStartPumping = 25; //sec
        int DelayCounter;
        int DelayValue;
        const double DelayInSeconds = 0.3; //sec задержка начала записи сигнала после выключения компрессора
        int HeartVisibleDelay = 50;
        int HeartVisibleCounter;
        bool HRVmode = false;

        double MaxPressure = 0;

        double MaxDerivValue;

        DeviceStatus DevStatus;

        public event Action<Message> WindowsMessageHandler;

        public Form1()
        {
            InitializeComponent();
            ValueToMmHg = new();
            ValueToMmHg.CoeffChanged += OnCoeffChanged;
            labHeart.Text = "♥";
            BufPanel = new BufferedPanel();
            BufPanel.MouseMove += BufPanel_MouseMove;
            Cfg = TTestConfig.GetConfig();
            if (Cfg.Maximized)
            {
                WindowState = FormWindowState.Maximized;
            }
            else
            {
                WindowState = FormWindowState.Normal;
                Width = Cfg.WindowWidth;
                Height = Cfg.WindowHeight;
            }
            numUDLeft.Value = Cfg.CoeffLeft;
            numUDRight.Value = Cfg.CoeffRight;
            numUDpressure.Value = Cfg.ToPressure;
            panelGraph.Dock = DockStyle.Fill;
            panelGraph.Controls.Add(BufPanel);
            BufPanel.Dock = DockStyle.Fill;
            BufPanel.Paint += bufferedPanel_Paint;
            VisirList = new List<int[]>();
            InitArraysForFlow();
            string ConnectionString = String.Empty;
            try
            {
                ConnectionString = File.ReadAllText("conectstr.txt");
            }
            catch (Exception)
            {
                MessageBox.Show("Connection string not found. The device cannot be connected");
                return;
            }
            USBPort = new USBSerialPort(this, Decomposer.BaudRate, ConnectionString);
            USBPort.ConnectionFailure += OnConnectionFailure;
            USBPort.ConnectionOk += OnConnectionOk;
            USBPort.Connect();
            DevStatus = new DeviceStatus();
        }

        #region [ Buttons Clicks ]
        private void button1_Click(object sender, EventArgs e)
        {
            USBPort.WriteByte((byte)CmdDevice.StartReading);
        }

        private void button2_Click(object sender, EventArgs e)
        {
            USBPort.WriteByte((byte)CmdDevice.StopReading);
        }

        private void butPressureMeasAbort_Click(object sender, EventArgs e)
        {
            DevStatus.ValveSlowClosed = false;
            DevStatus.ValveFastClosed = false;
            DevStatus.PumpIsOn = false;
            PumpStatus = PumpingStatus.Ready;
            USBPort.WriteByte((byte)CmdDevice.ValveFastOpen);
            USBPort.WriteByte((byte)CmdDevice.ValveSlowOpen);
            USBPort.WriteByte((byte)CmdDevice.PumpSwitchOff);
            PressureMeasStatus = PressureMeasurementStatus.Ready;
            PumpStatus = PumpingStatus.Ready;
            butStopRecord_Click(butPressureMeasAbort, e);
        }

        private void butValvesOpen_Click(object sender, EventArgs e)
        {
            butValve1Open_Click(this, EventArgs.Empty);
            butValve2Open_Click(this, EventArgs.Empty);
        }

        private void butValvesClose_Click(object sender, EventArgs e)
        {
            butValve1Close_Click(this, EventArgs.Empty);
            butValve2Close_Click(this, EventArgs.Empty);
        }

        private void butValve2Open_Click(object sender, EventArgs e)
        {
            DevStatus.ValveFastClosed = false;
            USBPort?.WriteByte((byte)CmdDevice.ValveFastOpen);
        }

        private void butValve2Close_Click(object sender, EventArgs e)
        {
            DevStatus.ValveFastClosed = true;
            USBPort.WriteByte((byte)CmdDevice.ValveFastClose);
        }

        private void butValve1Open_Click(object sender, EventArgs e)
        {
            DevStatus.ValveSlowClosed = false;
            USBPort?.WriteByte((byte)CmdDevice.ValveSlowOpen);
        }

        private void butValve1Close_Click(object sender, EventArgs e)
        {
            DevStatus.ValveSlowClosed = true;
            USBPort.WriteByte((byte)CmdDevice.ValveSlowClose);
        }

        private void butPumpOn_Click(object sender, EventArgs e)
        {
            DevStatus.PumpIsOn = true;
            USBPort.WriteByte((byte)CmdDevice.PumpSwitchOn);
        }

        private void butPumpOff_Click(object sender, EventArgs e)
        {
            DevStatus.PumpIsOn = false;
            USBPort.WriteByte((byte)CmdDevice.PumpSwitchOff);
        }

        private void butStopRecord_Click(object sender, EventArgs e)
        {
            labArrythmia.Text = Detector?.Arrhythmia.ToString();
            //            Detector.OnWaveDetected -= NewWaveDetected;
            Detector = null;
            progressBarRecord.Visible = false;
            Decomposer.OnDecomposePacketEvent -= OnPacketReceived;
            Decomposer.RecordStarted = false;
            TextWriter?.Dispose();

            CurrentFileSize = Decomposer.PacketCounter;
            labRecordSize.Text = "Record size : " + (CurrentFileSize / Decomposer.SamplingFrequency).ToString() + " s";
            UpdateScrollBar(CurrentFileSize);
            PressureMeasStatus = PressureMeasurementStatus.Ready;
            PumpStatus = PumpingStatus.Ready;
            butValvesOpen_Click(sender, EventArgs.Empty);
            ViewMode = true;
            timerPaint.Enabled = !ViewMode;
            timerRead.Enabled = false;
            if (sender == butPressureMeasAbort)
            {
                return;
            }
            PrepareData();
            BufPanel.Refresh();
            controlPanel.Refresh();
        }

        private void butStartMeas_Click(object sender, EventArgs e)
        {
            PumpingDia = 0;
            PumpingSys = 0;
            ValueToMmHg.ChangeCoeff("T");
            HRVmode = false;
            TextWriter = new StreamWriter(Cfg.DataDir + TmpDataFile);
            Decomposer.PacketCounter = 0;
            Decomposer.MainIndex = 0;
            Decomposer.RecordStarted = true;
            progressBarRecord.Visible = true;
            labSys.Text = "Sys : ";
            labDia.Text = "Dia : ";
            labPulse.Text = "Pulse : ";
            Detector = new WaveDetector();
            Detector.OnWaveDetected += NewWaveDetected;

            DevStatus.ValveSlowClosed = true;
            DevStatus.ValveFastClosed = true;
            DevStatus.PumpIsOn = true;
            PumpStatus = PumpingStatus.WaitingForLevel;
            PressureMeasStatus = PressureMeasurementStatus.Calibration;
            USBPort.WriteByte((byte)CmdDevice.StartReading);
        }

        private void butStartRecord_Click(object sender, EventArgs e) //HRV
        {
            HRVmode = true;
            TextWriter = new StreamWriter(Cfg.DataDir + TmpDataFile);
            Decomposer.PacketCounter = 0;
            Decomposer.MainIndex = 0;
            Decomposer.RecordStarted = true;
            progressBarRecord.Visible = true;
            Detector = new WaveDetector();
            Detector.OnWaveDetected += NewWaveDetected;
        }

        private void butFlow_Click(object sender, EventArgs e)
        {
            ViewMode = !ViewMode;
            if (!ViewMode)
            {
                InitArraysForFlow();
                hScrollBar1.Visible = false;
            }
        }

        private void butRefresh_Click(object sender, EventArgs e)
        {
            PrepareData();
            BufPanel.Refresh();
            controlPanel.Refresh();
        }

        private void butSaveFile_Click(object sender, EventArgs e)
        {
            saveFileDialog1.InitialDirectory = Cfg.DataDir.ToString();
            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                Cfg.DataDir = Path.GetDirectoryName(saveFileDialog1.FileName) + @"\";
                TTestConfig.SaveConfig(Cfg);
                CurrentFile = Path.GetFileName(saveFileDialog1.FileName);
                if (File.Exists(Cfg.DataDir + CurrentFile))
                {
                    File.Delete(Cfg.DataDir + CurrentFile);
                }
                File.Move(saveFileDialog1.InitialDirectory + TmpDataFile, Cfg.DataDir + CurrentFile);
                Text = "File : " + CurrentFile;
            }
        }

        private void butOpenFile_Click(object sender, EventArgs e)
        {
            openFileDialog1.FileName = "";
            openFileDialog1.InitialDirectory = Cfg.DataDir.ToString();
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                ViewMode = true;
                if (File.Exists(openFileDialog1.FileName))
                {
                    Cfg.DataDir = Path.GetDirectoryName(openFileDialog1.FileName) + @"\";
                    TTestConfig.SaveConfig(Cfg);
                    timerRead.Enabled = false;

                    CurrentFile = Path.GetFileName(openFileDialog1.FileName);
                    ValueToMmHg.ChangeCoeff(CurrentFile);
                    ReadFile(Cfg.DataDir + CurrentFile);
                }
            }
            Text = "File : " + CurrentFile;
        }

        private void butStartToPressure_Click(object sender, EventArgs e)
        {
            Cfg.ToPressure = numUDpressure.Value;
            butValvesClose_Click(this, EventArgs.Empty);
            butPumpOn_Click(this, EventArgs.Empty);
            PressureMeasStatus = PressureMeasurementStatus.Pumping;
        }

        private void butCalibr_Click(object sender, EventArgs e)
        {
            Decomposer.ZeroLine = Decomposer.tmpZero;
        }
        #endregion

        #region [ Controls Changed ]
        private void hScrollBar1_ValueChanged(object? sender, EventArgs e)
        {
            ViewShift = hScrollBar1.Value;
            BufPanel.Refresh();
        }

        private void trackBarAmp_ValueChanged(object? sender, EventArgs e)
        {
            double a = trackBarAmp.Value;
            ScaleY = Math.Pow(2, a / 2);
            BufPanel.Refresh();
        }
        private void numUDLeft_ValueChanged(object sender, EventArgs e)
        {
            Cfg.CoeffLeft = numUDLeft.Value;
        }

        private void numUDRight_ValueChanged(object sender, EventArgs e)
        {
            Cfg.CoeffRight = numUDRight.Value;
        }
        private void numUDpressure_ValueChanged(object sender, EventArgs e)
        {
            Cfg.ToPressure = numUDpressure.Value;
        }
        #endregion ]

        #region [ Timers Ticks ]
        private void timerStatus_Tick(object sender, EventArgs e)
        {
            labMeasurementStatus.Text = "Measurement status : " + PressureMeasStatus.ToString();
            labPumpStatus.Text = "Pumpung status : " + PumpStatus.ToString();

            timerRead.Enabled = !ViewMode;
            timerPaint.Enabled = !ViewMode;

            labValve1.Text = DevStatus.ValveSlowClosed ? "Valve 1 (Slow) : closed" : "Valve 1 (Slow) : opened";
            labValve2.Text = DevStatus.ValveFastClosed ? "Valve 2 (Fast) : closed" : "Valve 2 (Fast) : opened";
            labPump.Text = DevStatus.PumpIsOn ? "Pump : On" : "Pump : Off";
            butValveSlowClose.Enabled = !DevStatus.ValveSlowClosed;
            butValveSlowOpen.Enabled = DevStatus.ValveSlowClosed;
            butValveFastClose.Enabled = !DevStatus.ValveFastClosed;
            butValveFastOpen.Enabled = DevStatus.ValveFastClosed;
            butValvesOpen.Enabled = DevStatus.ValveFastClosed || DevStatus.ValveSlowClosed;
            butValvesClose.Enabled = !DevStatus.ValveFastClosed || !DevStatus.ValveSlowClosed;

            butPumpOn.Enabled = !DevStatus.PumpIsOn;
            butPumpOff.Enabled = DevStatus.PumpIsOn;

            if (Decomposer is null)
            {
                return;
            }
            butStartMeas.Enabled = !ViewMode && !Decomposer.RecordStarted!;
            butStopRecord.Enabled = Decomposer.RecordStarted;
            butSaveFile.Enabled = ViewMode && Decomposer.PacketCounter != 0;
            butFlow.Text = ViewMode ? "Start stream" : "Stop stream";
            if (USBPort == null)
            {
                labPort.Text = "Disconnected";
                ViewMode = true;
                return;
            }
            if (USBPort.PortHandle == null)
            {
                labPort.Text = "Disconnected";
                ViewMode = true;
                return;
            }
            if (USBPort.PortHandle.IsOpen)
            {
                labPort.Text = "Connected to " + USBPort.PortNames[USBPort.CurrentPort];
            }
            else
            {
                labPort.Text = "Disconnected";
            }
        }

        private void timerRead_Tick(object sender, EventArgs e)
        {
            if (USBPort?.PortHandle?.IsOpen == true)
            {
                Decomposer?.Decompos(USBPort, TextWriter);
            }
        }

        private void timerPaint_Tick(object sender, EventArgs e)
        {
            BufPanel.Refresh();
        }

        private void timerDetectRate_Tick(object sender, EventArgs e)
        {
            Decomposer.SamplingFrequency = Decomposer.PacketCounter / 10;
            timerDetectRate.Enabled = false;
            labelRate.Text = "Sample rate : " + Decomposer.SamplingFrequency.ToString();
        }
        #endregion

        private void InitArraysForFlow()
        {
            DataA = new DataArrays(ByteDecomposer.DataArrSize);
            Decomposer = new ByteDecomposerADS1115(DataA);
            Decomposer.OnDecomposePacketEvent += OnPacketReceived;
            Painter = new CurvesPainter(BufPanel, Decomposer);
        }

        private void OnConnectionOk()
        {
            USBPort.WriteByte((byte)CmdDevice.StartReading);
        }

        private void OnConnectionFailure(Exception obj)
        {
            ShowError(BPMError.Connection);
            ViewMode = true;
        }

        protected override void WndProc(ref Message m)
        {
            const int WM_DEVICECHANGE = 0x0219;
            if (m.Msg == WM_DEVICECHANGE)
            {
                WindowsMessageHandler?.Invoke(m);
            }
            base.WndProc(ref m);
        }

        private void ReadFile(string fileName)
        {
            int PatientDataCount = 7;

            string[] lines = File.ReadAllLines(fileName);
            lines = lines.Skip(PatientDataCount).ToArray();
            CurrentFileSize = lines.Length;
            labRecordSize.Text = "Record size : " + (CurrentFileSize / Decomposer.SamplingFrequency).ToString() + " s";
            UpdateScrollBar(CurrentFileSize);

            if (CurrentFileSize == 0)
            {
                ShowError(BPMError.ReadingFile);
                return;
            }
            DataA = DataArrays.CreateDataFromLines(lines);
            if (DataA == null)
            {
                ShowError(BPMError.ReadingFile);
                return;
            }
            DataA.CountViewArrays();
            PrepareData();
            BufPanel.Refresh();
        }

        private void PrepareData()
        {
            //Детектор - обнаружение пульсовых волн по производной
            WaveDetector WD = new();
            WD.Reset();
            for (int i = 0; i < CurrentFileSize; i++)
            {
                DataA.DebugArray[i] = WD.Detect(DataA.DerivArray, i);
            }

            labArrythmia.Text = WD.Arrhythmia.ToString();

            //Получение массива максимумов пульсаций давления (в окрестностях максимума производной)
            int[] ArrayOfWaveIndexes = DataProcessing.GetArrayOfWaveIndexes(DataA.PressureViewArray, WD.FiltredPoints.ToArray());
            double[] ArrayOfWaveAmplitudes = ArrayOfWaveIndexes.Select(x => DataA.PressureViewArray[x]).ToArray();
            DataProcessing.RemoveArtifacts(ArrayOfWaveAmplitudes);

            double MaximumAmplitude = ArrayOfWaveAmplitudes.Max();
            int XMaxIndex = Array.IndexOf(ArrayOfWaveAmplitudes, MaximumAmplitude);
            int XMax = ArrayOfWaveIndexes[XMaxIndex];

            //-----Второй проход с изменяемым порогом------------
            //-----До максимума 0.7, после максимума 0.55--------

            WD.Reset();
            for (int i = 0; i < CurrentFileSize; i++)
            {
                DataA.DebugArray[i] = WD.Detect(DataA.DerivArray, i, XMax);
            }

            //Получение массива максимумов пульсаций давления (в окрестностях максимума производной)
            ArrayOfWaveIndexes = DataProcessing.GetArrayOfWaveIndexes(DataA.PressureViewArray, WD.FiltredPoints.ToArray());
            ArrayOfWaveAmplitudes = ArrayOfWaveIndexes.Select(x => DataA.PressureViewArray[x]).ToArray();
            DataProcessing.RemoveArtifacts(ArrayOfWaveAmplitudes);

            MaximumAmplitude = ArrayOfWaveAmplitudes.Max();
            XMaxIndex = Array.IndexOf(ArrayOfWaveAmplitudes, MaximumAmplitude);
            XMax = ArrayOfWaveIndexes[XMaxIndex];
            //--------------------------------------------

            VisirList.Clear();
            VisirList.Add(ArrayOfWaveIndexes);

            double[] ArrLeftValues = new double[XMaxIndex];
            double[] ArrRightValues = new double[ArrayOfWaveIndexes.Length - XMaxIndex];
            Array.Copy(ArrayOfWaveAmplitudes, ArrLeftValues, ArrLeftValues.Length);
            Array.Copy(ArrayOfWaveAmplitudes, XMaxIndex, ArrRightValues, 0, ArrRightValues.Length);
            double[] ArrLeftSorted = ArrLeftValues.OrderBy(x => x).ToArray();
            double[] ArrRightSorted = ArrRightValues.OrderByDescending(x => x).ToArray();
            double[] ArrValues = ArrLeftSorted.Concat(ArrRightSorted).ToArray();

            //Построение огибающей максимумов пульсаций давления
            DataA.CountEnvelopeArray(ArrayOfWaveIndexes, ArrValues);

            labAF.Visible = Arrhythmia.AtrialFibrillation(ArrayOfWaveIndexes);

            //Подготовка массива для вычисления пульса
            int[] ArrayForPulse;
            if (HRVmode)
            {
                int DecreaseSize = 2; //Количество отбрасываемых интервалов справа и слева
                int TakeSize = ArrayOfWaveIndexes.Length - DecreaseSize * 2;
                ArrayForPulse = ArrayOfWaveIndexes.Skip(DecreaseSize).Take(TakeSize).ToArray();
            }
            else
            {
                int ArrayForPulseSize = 10;
                int shift = 6;
                ArrayForPulse = DataProcessing.GetSubArray(ArrayOfWaveIndexes, XMaxIndex - shift, XMaxIndex - shift + ArrayForPulseSize);
            }

            labPulse.Text = "Pulse : " + DataProcessing.GetPulseFromIndexesArray(ArrayForPulse, Decomposer.SamplingFrequency).ToString();
            labNumOfWaves.Text = "Waves detected : " + (ArrayOfWaveIndexes.Length).ToString();

            if (HRVmode)
            {
                FormHRV formHRV = new(ArrayOfWaveIndexes, Decomposer.SamplingFrequency);
                formHRV.ShowDialog();
                formHRV.Dispose();
                return;
            }

            double PSys = 0;
            double PDia = 0;
            int indexPSys = 0;
            int indexPDia = 0;
            double ValueSys = MaximumAmplitude * (double)Cfg.CoeffLeft;
            double ValueDia = MaximumAmplitude * (double)Cfg.CoeffRight;

            //Определение систолического давления (влево от Max)
            for (int i = XMax; i >= 0; i--)
            {
                if (DataA.EnvelopeArray[i] < ValueSys)
                {
                    PSys = DataA.DCArray[i];
                    indexPSys = i;
                    break;
                }
            }
            if (PSys == 0)
            {
                PSys = DataA.DCArray[0];
                ShowError(BPMError.Sys);
            }
            //Определение диастолического давления (вправо от Max)
            for (int i = XMax; i < DataA.Size; i++)
            {
                if (DataA.EnvelopeArray[i] < ValueDia)
                {
                    PDia = DataA.DCArray[i];
                    indexPDia = i;
                    break;
                }
            }
            if (PDia == 0)
            {
                PDia = DataA.DCArray[DataA.Size - 1];
                ShowError(BPMError.Dia);
            }

            int[] ArrayOfPoints = { indexPSys, XMax, indexPDia };
            VisirList.Add(ArrayOfPoints);

            labSys.Text = "Sys : " + ValueToMmHg.Convert(PSys).ToString();
            labDia.Text = "Dia : " + ValueToMmHg.Convert(PDia).ToString();
        }

        private void bufferedPanel_Paint(object sender, PaintEventArgs e)
        {
            if (DataA == null)
            {
                return;
            }
            var ArrayList = new List<double[]>();
            if (ViewMode)
            {
                ArrayList.Add(DataA.PressureViewArray);
                ArrayList.Add(DataA.DerivArray);
                ArrayList.Add(DataA.DebugArray);
                ArrayList.Add(DataA.EnvelopeArray);//Последняя кривая в списке жирная
            }
            else
            {
                ArrayList.Add(DataA.PressureViewArray);
                ArrayList.Add(DataA.DerivArray);
            }
            Painter.Paint(ViewMode, ViewShift, ArrayList, VisirList, ScaleY, MaxValue, e);
        }

        private void UpdateScrollBar(int size)
        {
            hScrollBar1.Maximum = size;
            hScrollBar1.LargeChange = panelGraph.Width - 50;
            hScrollBar1.SmallChange = panelGraph.Width / 10;
            hScrollBar1.AutoSize = true;
            hScrollBar1.Value = 0;
            hScrollBar1.Visible = hScrollBar1.Maximum > hScrollBar1.Width;
        }

        private void Form1_FormClosing(object? sender, FormClosingEventArgs e)
        {
            butValvesOpen_Click(this, EventArgs.Empty);
            Cfg.Maximized = WindowState == FormWindowState.Maximized;
            Cfg.WindowWidth = Width;
            Cfg.WindowHeight = Height;
            TTestConfig.SaveConfig(Cfg);
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            if (DataA == null)
            {
                return;
            }
            if (!ViewMode)
            {
                return;
            }
            DataA.CountViewArrays();
            BufPanel.Refresh();
        }

        private void BufPanel_MouseMove(object sender, MouseEventArgs e)
        {
            if (!ViewMode)
            {
                return;
            }
            if (DataA == null)
            {
                return;
            }
            double x = e.X + ViewShift;
            if (x > DataA.Size - 1)
            {
                return;
            }

            int index = e.X + ViewShift;
            double sec = index / Decomposer.SamplingFrequency;
            labelX.Text = String.Format("X : {0}, Time {1:f2} s ", index, sec);
            labY0.Text = String.Format("PressureViewArray : {0:f0}", DataA.PressureViewArray[index]);
            labY1.Text = String.Format("RealTimeArray : {0:f0}", DataA.RealTimeArray[index]);
            labY2.Text = String.Format("DCArray : {0:f0}", DataA.DCArray[index]) + "  " +
                                        ValueToMmHg.Convert(DataA.DCArray[index]).ToString();
        }

        private void OnCoeffChanged(object senger, ConverterValueToMmHgEventArgs e)
        {
            labCoeff.Text = "Coeff : " + e.Coeff.ToString();
        }

        int PumpingDia;
        int PumpingSys;
        private void NewWaveDetected(object sender, WaveDetectorEventArgs e)
        {
            double StopMeasCoeff = 0.5;
            HeartVisibleCounter = HeartVisibleDelay;
            string text = e.WaveCount.ToString() + " " + e.Interval.ToString() + " " + e.Amplitude.ToString("0.0");
            labNumOfWaves.Text = "Waves detected: " + text;

            switch (PressureMeasStatus)
            {
                case PressureMeasurementStatus.Pumping:
                    if (PumpingDia == 0)
                    {
                        PumpingDia = ValueToMmHg.Convert(DataA.DCArray[e.Index]);
                    }
                    else
                    {
                        PumpingSys = ValueToMmHg.Convert(DataA.DCArray[e.Index]);
                    }
                    
                    switch (PumpStatus)
                    {
                        case PumpingStatus.WaitingForLevel:
                            if (e.Amplitude > Decomposer.StartSearchMaxLevel)
                            {
                                PumpStatus = PumpingStatus.MaximumSearch;
                            }
                            break;
                        case PumpingStatus.MaximumSearch:
                            MaxDerivValue = Math.Max(MaxDerivValue, e.Amplitude);
                            if (MaxDerivValue > e.Amplitude)
                            {
                                PumpStatus = PumpingStatus.MaximumFound;
                                MomentMaxFound = (int)Decomposer.MainIndex;
                            }
                            break;
                        case PumpingStatus.MaximumFound:
                            if (e.Amplitude < Decomposer.StopPumpingLevel && CurrentPressure > MinPressure)
                            {
                                StopPumping("Stop pumping level reached");
                            }
                            break;
                    }
                    break;
                case PressureMeasurementStatus.Measurement:
//                    labPressPumping.Text = PumpingSys.ToString() + "/" + PumpingDia.ToString();
                    MaxDerivValue = Math.Max(e.Amplitude, MaxDerivValue);
                    if (e.Amplitude < MaxDerivValue * StopMeasCoeff)
                    {
                        DevStatus.ValveSlowClosed = false; 
                        DevStatus.ValveFastClosed = false;
                        PressureMeasStatus = PressureMeasurementStatus.Ready;
                        USBPort.WriteByte((byte)CmdDevice.ValveFastOpen);
                        USBPort.WriteByte((byte)CmdDevice.StopReading);
                        butStopRecord_Click(this, EventArgs.Empty);
                    }
                    break;
                default:
                    break;
            }
        }

        private void OnPacketReceived(object sender, PacketEventArgs e)
        {
            labZero.Text = "Zero : " + Decomposer.ZeroLine.ToString();
            labHeart.Visible = HeartVisibleCounter != 0;
            if (HeartVisibleCounter != 0)
            {
                HeartVisibleCounter--;
            }

            uint currentIndex = (e.MainIndex - 1) & (ByteDecomposer.DataArrSize - 1);
            CurrentPressure = ValueToMmHg.Convert(e.RealTimeValue);

            double aver = 0;
            int averSize = 250;
            for (int i = 0; i < averSize; i++)
            {
                long index = (currentIndex - i) & (ByteDecomposer.DataArrSize - 1);
                aver += DataA.RealTimeArray[index];
            }
            aver = aver / averSize;
            DataA.DerivArray[currentIndex] = DataProcessing.GetDerivative(DataA.PressureViewArray, currentIndex);
            MaxPressure = (int)Math.Max(DataA.RealTimeArray[currentIndex], MaxPressure);
            labCurrentPressure.Text = "Current : " +
                                        CurrentPressure.ToString() +
                                        " RealTime : " +
                                        DataA.RealTimeArray[currentIndex].ToString() + " " +
                                        Math.Round(aver).ToString();
            if (Decomposer.RecordStarted)
            {
                if (PressureMeasStatus == PressureMeasurementStatus.Calibration)
                {
                    Decomposer.ZeroLine = Decomposer.tmpZero;
                    USBPort.WriteByte((byte)CmdDevice.ValveFastClose);
                    USBPort.WriteByte((byte)CmdDevice.ValveSlowClose);
                    USBPort.WriteByte((byte)CmdDevice.PumpSwitchOn);
                    PressureMeasStatus = PressureMeasurementStatus.Pumping;
                    PumpStatus = PumpingStatus.WaitingForLevel;
                }
                labRecordSize.Text = "Record size : " + (e.PacketCounter / Decomposer.SamplingFrequency).ToString() + " c";
                DataA.DebugArray[currentIndex] = (int)Detector.Detect(DataA.DerivArray, (int)currentIndex);
                if (CurrentPressure > MaxAllowablePressure)
                { 
                    StopPumping("Max Allowable Pressure");
                }
                if (PumpStatus == PumpingStatus.WaitingForLevel)
                {
                    bool timeout = e.PacketCounter / Decomposer.SamplingFrequency > MaxTimeAfterStartPumping;
                    if (timeout)
                    {
                        butPressureMeasAbort_Click(this, EventArgs.Empty);
                        ShowError(BPMError.AirLeak);
                    }
                }
                if (PumpStatus == PumpingStatus.MaximumFound)
                {
                    int Index = e.PacketCounter;
                    bool timeout = (Index - MomentMaxFound) / Decomposer.SamplingFrequency > MaxTimeAfterMaxFound;
                    if (timeout && CurrentPressure > MinPressure)
                    {
                        StopPumping("Timeout");
                    }
                }
                if (PressureMeasStatus == PressureMeasurementStatus.Delay)
                {
                    DelayCounter++;
                    if (DelayCounter > DelayValue)
                    {
                        PressureMeasStatus = PressureMeasurementStatus.Measurement;
                        AfterDelay();
                    }
                }
            }
        }

        private void StopPumping(string mess)
        {
            MaxPressureValue = ValueToMmHg.Convert(DataA.DCArray[Decomposer.MainIndex]);
            DevStatus.PumpIsOn = false;
            DevStatus.ValveSlowClosed = false;
            labStopPumpingReason.Text = mess;
            PumpStatus = PumpingStatus.Ready;
            USBPort.WriteByte((byte)CmdDevice.PumpSwitchOff);
            PressureMeasStatus = PressureMeasurementStatus.Delay;
            DelayCounter = 0;
            DelayValue = (int)(Decomposer.SamplingFrequency * DelayInSeconds);
        }

        private void AfterDelay()
        {
            PumpStatus = PumpingStatus.Ready;
            Decomposer.PacketCounter = 0;
            timerDetectRate.Enabled = true;
            Decomposer.MainIndex = 0;
            MaxDerivValue = 0;
            Detector?.Reset();
            USBPort.WriteByte((byte)CmdDevice.ValveSlowOpen);
            PressureMeasStatus = PressureMeasurementStatus.Measurement;
            timerDetectRate.Enabled = true;
            labPressPumping.Text = PumpingSys.ToString() + " / " + PumpingDia.ToString() + " Reached pressure " + MaxPressureValue;
        }

        private static void ShowError(BPMError error)
        {
            string errorText = error switch
            {
                BPMError.AirLeak     => "Air leak",
                BPMError.ReadingFile => "Reading file error",
                BPMError.Connection  => "Connection failure",
                BPMError.Sys         => "Systolic pressure error",
                BPMError.Dia         => "Diastolic pressure error",
                _ => "",
            };
            MessageBox.Show(errorText, "Error");
        }
    }
}