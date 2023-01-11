using TTestApp.Decomposers;

namespace TTestApp
{
    internal class CurvesPainter
    {
        private readonly Control _control;
        private readonly ByteDecomposer _decomposer;
        private readonly Color[] curveColors = { Color.Red, Color.Blue, Color.Green, Color.Brown, Color.Aquamarine };
        private readonly Color[] visirsColors = { Color.LightGray, Color.Brown, Color.Chocolate };

        public CurvesPainter(Control control, ByteDecomposer decomposer)
        {
            _control = control;
            _decomposer = decomposer;
        }   

        public void Paint(
            bool viewMode,
            int viewShift,
            List<double[]> data,
            List<int[]> visirs,
            double scaleY,
            int maxSize,
            PaintEventArgs e)
        {
            if (data == null)
            {
                return;
            }
            if (data.Count == 0)
            {
                return;
            }
            float tension = 0.1F;
            var R0 = new Rectangle(0, 0, _control.Width - 1, _control.Height - 1);
            var pen0 = new Pen(Color.Black, 1);
            e.Graphics.Clear(Color.White);
            e.Graphics.DrawRectangle(pen0, R0);
            e.Graphics.DrawLine(pen0, 0, R0.Height / 2, R0.Width, R0.Height / 2);
            if (!viewMode)
            {
                for (int i = 0; i < data.Count; i++)
                {
                    var pen = new Pen(curveColors[i], 1);
                    Point[] OutArray = ViewArrayMaker.MakeArray(
                                                            _control,
                                                            data[i],
                                                            _decomposer.MainIndex,
                                                            maxSize,
                                                            scaleY);
                    e.Graphics.DrawCurve(pen, OutArray, tension);
                    pen.Dispose();
                }
            }
            else //Режим просмотра
            {
                for (int i = 0; i < data.Count; i++)
                {
                    if (data[i] == null) break;
                    int width = 1;
                    if (i == data.Count - 1)
                    {
                        width = 2;
                    }

                    var pen = new Pen(curveColors[i], width);

                    Point[] OutArray = ViewArrayMaker.MakeArrayForView(
                                                                    _control,
                                                                    data[i],
                                                                    viewShift,
                                                                    maxSize,
                                                                    scaleY,
                                                                    1);
                    try
                    {
                        e.Graphics.DrawCurve(pen, OutArray, tension);
                    }
                    catch (Exception)
                    {
                    }
                    pen.Dispose();
                }
                int X1 = viewShift;
                int X2 = _control.Width + viewShift;
                for (int i = 0; i < visirs.Count; i++)
                {
                    if (visirs[i] == null) break;
                    var pen = new Pen(visirsColors[i], 1);
                    for (int j = 0; j < visirs[i].Length; j++)
                    {
                        if (visirs[i][j] > X1 && visirs[i][j] < X2)
                        {
                            e.Graphics.DrawLine(pen, visirs[i][j] - viewShift, 0, visirs[i][j] - viewShift, _control.Width);
                        }
                    }
                    pen.Dispose();
                }
                int index = 1;
                int timeTickSize = 10;
                while (index * _decomposer.SamplingFrequency < _control.Width)
                {
                    e.Graphics.DrawLine(
                        pen0,
                        index * _decomposer.SamplingFrequency,
                        _control.Height,
                        index * _decomposer.SamplingFrequency,
                        _control.Height - timeTickSize);
                    index++;
                }
            }
            pen0.Dispose();
        }
    }
}
