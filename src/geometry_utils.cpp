#include "geometry_utils.h"

void Normalise(XYZ *r) {
    double norm = sqrt(pow(r->x, 2) + pow(r->y, 2) + pow(r->z, 2));
    r->x /= norm;
    r->y /= norm;
    r->z /= norm;
}

/*
   Rotate a point p by angle theta around an arbitrary axis r
   Return the rotated point.
   Positive angles are anticlockwise looking down the axis
   towards the origin.
   Assume right hand coordinate system.
*/
XYZ ArbitraryRotate(XYZ p,double theta,XYZ r)
{
   XYZ q = {0.0,0.0,0.0};
   double costheta,sintheta;

   Normalise(&r);
   costheta = cos(theta);
   sintheta = sin(theta);

   q.x += (costheta + (1 - costheta) * r.x * r.x) * p.x;
   q.x += ((1 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
   q.x += ((1 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

   q.y += ((1 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
   q.y += (costheta + (1 - costheta) * r.y * r.y) * p.y;
   q.y += ((1 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

   q.z += ((1 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
   q.z += ((1 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
   q.z += (costheta + (1 - costheta) * r.z * r.z) * p.z;

   return(q);
}


/*
   Rotate a point p by angle theta around an arbitrary line segment p1-p2
   Return the rotated point.
   Positive angles are anticlockwise looking down the axis
   towards the origin.
   Assume right hand coordinate system.  
*/
XYZ ArbitraryRotate2(XYZ p,double theta,XYZ p1,XYZ p2)
{
   XYZ q = {0.0,0.0,0.0};
   double costheta,sintheta;
   XYZ r;

   r.x = p2.x - p1.x;
   r.y = p2.y - p1.y;
   r.z = p2.z - p1.z;
   p.x -= p1.x;
   p.y -= p1.y;
   p.z -= p1.z;
   Normalise(&r);

   costheta = cos(theta);
   sintheta = sin(theta);

   q.x += (costheta + (1 - costheta) * r.x * r.x) * p.x;
   q.x += ((1 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
   q.x += ((1 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

   q.y += ((1 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
   q.y += (costheta + (1 - costheta) * r.y * r.y) * p.y;
   q.y += ((1 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

   q.z += ((1 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
   q.z += ((1 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
   q.z += (costheta + (1 - costheta) * r.z * r.z) * p.z;

   q.x += p1.x;
   q.y += p1.y;
   q.z += p1.z;
   return(q);
}

Eigen::Vector3f RotatePoint(Eigen::Vector3f const &pt, double theta, Eigen::Vector3f const &axis_p1, Eigen::Vector3f const &axis_p2) {
    XYZ p = {pt.x(), pt.y(), pt.z()};
    XYZ p1 = {axis_p1.x(), axis_p1.y(), axis_p1.z()};
    XYZ p2 = {axis_p2.x(), axis_p2.y(), axis_p2.z()};
    XYZ e = ArbitraryRotate2(p, theta, p1, p2);
    Eigen::Vector3f res (e.x, e.y, e.z);
    return res;
}

Eigen::Vector4f VectorToQuaternion(Eigen::Vector3f const &vDirection)
        {
            // Step 1. Setup basis vectors describing the rotation given the input vector and assuming an initial up direction of (0, 1, 0)
            Eigen::Vector3f vUp (0.0, 1.0, 0.0);           // Y Up vector
            Eigen::Vector3f vRight = vUp.cross(vDirection);    // The perpendicular vector to Up and Direction
            vUp = vDirection.cross(vRight);            // The actual up vector given the direction and the right vector

            double w = sqrt(1.0 + vRight.x() + vUp.y() + vDirection.z()) / 2.0;
            double dfWScale = w * 4.0;
            double x = (vDirection.y() - vUp.z()) / dfWScale;
            double y = (vRight.z() - vDirection.x()) / dfWScale;
            double z = (vUp.x() - vRight.y()) / dfWScale;

            Eigen::Vector4f res (x, y, z, w);

            return res;
        }

Eigen::Vector4f VectorToQuaternion(pcl::ModelCoefficients::ConstPtr v) {

    Eigen::Vector3f vDirection(v->values.at(0), v->values.at(1), v->values.at(2));
    return VectorToQuaternion(vDirection);
}

/*
public Quaternion GenerateRotationFromDirectionVector(Vector3 vDirection)
        {
            // Step 1. Setup basis vectors describing the rotation given the input vector and assuming an initial up direction of (0, 1, 0)
            Vector3 vUp = new Vector3(0, 1.0f, 0.0f);           // Y Up vector
            Vector3 vRight = Vector3.Cross(vUp, vDirection);    // The perpendicular vector to Up and Direction
            vUp = Vector3.Cross(vDirection, vRight);            // The actual up vector given the direction and the right vector

            // Step 2. Put the three vectors into the matrix to bulid a basis rotation matrix
            // This step isnt necessary, but im adding it because often you would want to convert from matricies to quaternions instead of vectors to quaternions
            // If you want to skip this step, you can use the vector values directly in the quaternion setup below
            Matrix mBasis = new Matrix(vRight.X, vRight.Y, vRight.Z, 0.0f,
                                        vUp.X, vUp.Y, vUp.Z, 0.0f,
                                        vDirection.X, vDirection.Y, vDirection.Z, 0.0f,
                                        0.0f, 0.0f, 0.0f, 1.0f);

            // Step 3. Build a quaternion from the matrix
            Quaternion qrot = new Quaternion();
            qrot.W = (float)Math.Sqrt(1.0f + mBasis.M11 + mBasis.M22 + mBasis.M33) / 2.0f;
            double dfWScale = qrot.W * 4.0;
            qrot.X = (float)((mBasis.M32 - mBasis.M23) / dfWScale);
            qrot.Y = (float)((mBasis.M13 - mBasis.M31) / dfWScale);
            qrot.Z = (float)((mBasis.M21 - mBasis.M12) / dfWScale);

            return qrot;
        }
 */