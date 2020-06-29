# 3d_circle_estimation
3D circle estimation from its perspective projection.
This project is developed to test different methods from to estimate 3D pose of a 3D circle from its perspective projection.

Requried libraries to build to project: 
	- wxWidgets (for GUI), 
	- Eigen3 (for linear algebra), 
	- CeresSolver (for non-linear optimization)

Usage:
	- Draw at least one ellipse in the drawing area with 3 click for each ellipse.
		- 2 left click to draw the major-axis of the ellipse
		- 1 right click to draw the minor-axis of the ellipse and finalize the ellipse draw.
	- Use Estimate menu to compute estimated 3D circles whose projections are drawn ellipses.
		- 3 different algorithms are implemented from 3 papers.
			- Johan Philip, An algorithm for determining the position of a circle in 3D from its perspective 2D projection - 1997
			- M. Ferri, F. Mangili and G.Viano, Projective Pose Estimation of Linear and Quadratic Primitives in Monocular Computer Vision- 1993
			- Reza Safaee-Rad, Ivo Tchoukanov, Kenneth Carless Smith, and Bensiyon Benhabib, Three Dimensional Location Estimation of Circular Features for machine vision - 1992
	- A test 3D circle is projected in the main drawing area. 
