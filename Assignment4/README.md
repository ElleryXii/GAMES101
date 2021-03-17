# Bézier curve
Defined by control points.
# De Casteljau algorithm
Let ```t``` be a point on the Bézier curve defined by control point ```b_0```  to ```b_n```. Intepolate the value of point ```t``` on the segment ```b_0``` - ```b_1```, ```b_1``` - ```b_2```...```b_n-1```-```b_n```, etc. Connect those points, repeat the process on the new segments, until you have one point left. That is the value of the Bézier curve on point ```t```.
![example](/Assignment4/DeCasteljauExample.png)