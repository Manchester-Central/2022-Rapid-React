//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat cvRectangleOutput;
	Point newPoint0Output;
	Point newPoint1Output;
}

//
// Steps
//

Step CV_rectangle0
{
    Mat cvRectangleSrc = source0;
    Point cvRectanglePt1 = newPoint1Output;
    Point cvRectanglePt2 = newPoint0Output;
    Scalar cvRectangleColor = (0.0, 0.0, 0.0, 0.0);
    Double cvRectangleThickness = 0;
    LineType cvRectangleLinetype = LINE_AA;
    Double cvRectangleShift = 0;

    cvRectangle(cvRectangleSrc, cvRectanglePt1, cvRectanglePt2, cvRectangleColor, cvRectangleThickness, cvRectangleLinetype, cvRectangleShift, cvRectangleOutput);
}

Step New_Point0
{
    Double newPoint0X = 200.0;
    Double newPoint0Y = 220.0;

    newPoint(newPoint0X, newPoint0Y, newPoint0Output);
}

Step New_Point1
{
    Double newPoint1X = 160.0;
    Double newPoint1Y = 1.0;

    newPoint(newPoint1X, newPoint1Y, newPoint1Output);
}




