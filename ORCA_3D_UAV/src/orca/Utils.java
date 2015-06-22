package orca;

import sim.util.Double3D;

public class Utils{
	
	public static <T> void swap (T a, T b) {
		T temp=a;
		a=b;
		b=temp;	
	}
	
	/**
	 * \relates  Double3D
	 * \brief    Computes the cross product of the specified three-dimensional vectors.
	 * \param    vector1  The first vector with which the cross product should be computed.
	 * \param    vector2  The second vector with which the cross product should be computed.
	 * \return   The cross product of the two specified vectors.
	 */
	public static Double3D cross(Double3D vector1, Double3D vector2)
	{
		return new Double3D(vector1.y * vector2.z - vector1.z * vector2.y, vector1.z * vector2.x - vector1.x * vector2.z, vector1.x * vector2.y - vector1.y * vector2.x);
	}
	
	public static double Sq(double d)
	{
		return d*d;
	}


}
