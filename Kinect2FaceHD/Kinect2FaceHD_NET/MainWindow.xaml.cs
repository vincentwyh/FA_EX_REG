using System;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using System.Windows.Shapes;
using System.Collections.Generic;
using System.Windows.Controls;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows.Media.Imaging;



namespace Kinect2FaceHD_NET
{
    /// <summary>
    /// Testing
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// Radius of drawn hand circles
        private const double HandSize = 30;

        /// Thickness of drawn joint lines
        private const double JointThickness = 3;

        /// Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;

        /// Constant for clamping Z values of camera space points from being negative
        private const float InferredZPositionClamp = 0.1f;

        /// Brush used for drawing hands that are currently tracked as closed
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// Brush used for drawing hands that are currently tracked as opened
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// Brush used for drawing joints that are currently tracked
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// Brush used for drawing joints that are currently inferred
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// Pen used for drawing bones that are currently inferred
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// Drawing group for body rendering output
        private DrawingGroup drawingGroup;

        /// Drawing image that we will display
        private DrawingImage imageSource;

        /// Coordinate mapper to map one type of point to another
        private CoordinateMapper coordinateMapper = null;

        /// Array for the bodies
        private Body[] bodies = null;

        /// definition of bones
        private List<Tuple<JointType, JointType>> bones;

        /// Width of display (depth space)
        private int displayWidth;

        /// Height of display (depth space)
        private int displayHeight;

        /// List of colors for each body tracked
        private List<Pen> bodyColors;

        /// Current status text to display
        private string statusText = null;

        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        public event PropertyChangedEventHandler PropertyChanged;

        /// Gets the bitmap to display
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// Gets or sets the current status text to display
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }


        
        private KinectSensor _sensor = null;

        private BodyFrameSource _bodySource = null;

        private BodyFrameReader _bodyReader = null;

        private HighDefinitionFaceFrameSource _faceSource = null;

        private HighDefinitionFaceFrameReader _faceReader = null;

        private FaceAlignment _faceAlignment = null;

        private FaceModel _faceModel = null;

        private List<Ellipse> _points = new List<Ellipse>();





        public MainWindow()
        {
            // one sensor is currently supported
            this._sensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this._sensor.CoordinateMapper;
           
            // get the depth (display) extents
            FrameDescription frameDescription = this._sensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this._bodyReader = this._sensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this._sensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this._sensor.Open();

            // set the status text
            this.StatusText = this._sensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (this._bodyReader != null)
            {
                this._bodyReader.FrameArrived += this.Reader_FrameArrived;
            }

            if (_sensor != null)
            {
                _bodySource = _sensor.BodyFrameSource;
                _bodyReader = _bodySource.OpenReader();
                _bodyReader.FrameArrived += BodyReader_FrameArrived;

                _faceSource = new HighDefinitionFaceFrameSource(_sensor);

                _faceReader = _faceSource.OpenReader();
                _faceReader.FrameArrived += FaceReader_FrameArrived;

                _faceModel = new FaceModel();
                _faceAlignment = new FaceAlignment();

                _sensor.Open();
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (_faceModel != null)
            {
                _faceModel.Dispose();
                _faceModel = null;
            }

            if (this._bodyReader != null)
            {
                // BodyFrameReader is IDisposable
                this._bodyReader.Dispose();
                this._bodyReader = null;
            }

            if (this._sensor != null)
            {
                this._sensor.Close();
                this._sensor = null;
            }

            GC.SuppressFinalize(this);
        }

        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    Body[] bodies = new Body[frame.BodyCount];
                    frame.GetAndRefreshBodyData(bodies);

                    Body body = bodies.Where(b => b.IsTracked).FirstOrDefault();

                    if (!_faceSource.IsTrackingIdValid)
                    {
                        if (body != null)
                        {
                            _faceSource.TrackingId = body.TrackingId;
                        }
                    }
                }
            }
        }

        private void FaceReader_FrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null && frame.IsFaceTracked)
                {
                    frame.GetAndRefreshFaceAlignmentResult(_faceAlignment);
                    UpdateFacePoints();
                }
            }
        }

        private void UpdateFacePoints()
        {
            if (_faceModel == null) return;

            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            if (vertices.Count > 0)
            {
                if (_points.Count == 0)
                {
                    for (int index = 0; index < vertices.Count; index++)
                    {
                        Ellipse ellipse = new Ellipse
                        {
                            Width = 2.0,
                            Height = 2.0,
                            Fill = new SolidColorBrush(Colors.Blue)
                        };

                        _points.Add(ellipse);
                    }

                    foreach (Ellipse ellipse in _points)
                    {
                        canvas.Children.Add(ellipse);
                    }
                }

                for (int index = 0; index < vertices.Count; index++)
                {
                    CameraSpacePoint vertice = vertices[index];
                    DepthSpacePoint point = _sensor.CoordinateMapper.MapCameraPointToDepthSpace(vertice);

                    /*CameraSpacePoint MouthUpperlipMidtopCameraPoint = vertices[(int)HighDetailFacePoints.MouthUpperlipMidtop];
                    float x1 = MouthUpperlipMidtopCameraPoint.X;
                    float y1 = MouthUpperlipMidtopCameraPoint.Y;
                    float z1 = MouthUpperlipMidtopCameraPoint.Z;

                    CameraSpacePoint MouthLowerlipMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.MouthLowerlipMidbottom];
                    float x2 = MouthLowerlipMidbottomCameraPoint.X;
                    float y2 = MouthLowerlipMidbottomCameraPoint.Y;
                    float z2 = MouthLowerlipMidbottomCameraPoint.Z;

                    var LipsWidthDistance = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));*/

                    if (float.IsInfinity(point.X) || float.IsInfinity(point.Y)) return;

                    Ellipse ellipse = _points[index];

                    Canvas.SetLeft(ellipse, point.X);
                    Canvas.SetTop(ellipse, point.Y);
                }
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this._sensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        /*private void CalculateEyeHeightDistance()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint LefteyeMidtopCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidtop];
            float x1 = LefteyeMidtopCameraPoint.X;
            float y1 = LefteyeMidtopCameraPoint.Y;
            float z1 = LefteyeMidtopCameraPoint.Z;

            CameraSpacePoint LefteyeMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidbottom];
            float x2 = LefteyeMidbottomCameraPoint.X;
            float y2 = LefteyeMidbottomCameraPoint.Y;
            float z2 = LefteyeMidbottomCameraPoint.Z;

            var EyeWideDistance = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));
        }

        private void CalculateEyeBrowDistance()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint LefteyebrowCameraPoint = vertices[(int)HighDetailFacePoints.LefteyebrowCenter];
            float x1 = LefteyebrowCameraPoint.X;
            float y1 = LefteyebrowCameraPoint.Y;
            float z1 = LefteyebrowCameraPoint.Z;

            CameraSpacePoint LefteyeMidtopCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidtop];
            float x2 = LefteyeMidtopCameraPoint.X;
            float y2 = LefteyeMidtopCameraPoint.Y;
            float z2 = LefteyeMidtopCameraPoint.Z;

            var EyeBrowDistance = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));
            Console.WriteLine("{0}", EyeBrowDistance);
        }

        private void CalculateLipsWidthDistance()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint MouthUpperlipMidtopCameraPoint = vertices[(int)HighDetailFacePoints.MouthUpperlipMidtop];
            float x1 = MouthUpperlipMidtopCameraPoint.X;
            float y1 = MouthUpperlipMidtopCameraPoint.Y;
            float z1 = MouthUpperlipMidtopCameraPoint.Z;

            CameraSpacePoint MouthLowerlipMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.MouthLowerlipMidbottom];
            float x2 = MouthLowerlipMidbottomCameraPoint.X;
            float y2 = MouthLowerlipMidbottomCameraPoint.Y;
            float z2 = MouthLowerlipMidbottomCameraPoint.Z;

            var LipsWidthDistance = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));
            Console.WriteLine("{0}", LipsWidthDistance);
        }*/
    }
}