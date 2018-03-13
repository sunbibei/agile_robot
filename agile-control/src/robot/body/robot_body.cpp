/*
 * robot_body.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#include <robot/body/robot_body.h>
#include <robot/leg_robot.h>

namespace agile_control {

bool RobotBody::auto_init() {
  if (!MathBody::auto_init()) return false;

  LegRobot::instance()->body_iface_ = this;
  return true;
}

}


#ifdef XXX
namespace agile_control {



#define G (9.8)

RobotBody::RobotBody() {
 ;
}

//bool RobotBody::init() {
//
//  return true;
//}

 RobotBody::~RobotBody() {
   ;
 }

///! The translation of robot against the world frame
void RobotBody::translation(EV3&) /*= 0*/ {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
}

///! The rotation of robot against the world frame
void RobotBody::rotation(Eigen::Quaterniond&) /*= 0*/ {
  LOG_ERROR << "Call the 'rotation' which has does not complemented.";
}

///! The velocity of robot against the world frame
void RobotBody::velocity(EV3& v)  /*= 0*/ {
  LOG_ERROR << "Call the 'velocity' which has does not complemented.";
}

///! The centre of gravity of robot
void RobotBody::cog(EV3&)         /*= 0*/ {
  LOG_ERROR << "Call the 'cog' which has does not complemented.";
}



void RobotBody::setExecuteDuration(const double& duration)
{
  duration_ = duration;
}

void RobotBody::setCogThreshold(const double& threshold)
{
  cog_threshold_ = threshold;
}

//translate CoG to ZMP point  
void RobotBody::calZmpPos()
{
  body_zmp_(0) = body_cog_(0) - body_cog_(2)/(body_acc(2)+G)*body_acc(0);
  body_zmp_(1) = body_cog_(1) - body_cog_(2)/(body_acc(2)+G)*body_acc(1);
  body_zmp_(2) = body_cog_(2);
}
//三角形面积S=√[p(p-a)(p-b)(p-c)]
//其中p=(a+b+c)/2.(海伦公式)内切圆半径r=2S/(a+b+c)外接圆半径R=abc/4S
float RobotBody::calInscribedCircleRadius(const EV3& A, const EV3& B, const EV3& C)
{
  float c = sqrt(pow((A(0)-B(0)),2) + pow((A(1)-B(1)),2));
  float b = sqrt(pow((A(0)-C(0)),2) + pow((A(1)-C(1)),2));
  float a = sqrt(pow((C(0)-B(0)),2) + pow((C(1)-B(1)),2));
  float p = (a+b+c)/2.0;
  float s = sqrt(p*(p-a)*(p-b)*(p-c));

  return 2.0*s/(a+b+c);
}

EV3 RobotBody::calInnerHeart(const EV3& A, const EV3& B, const EV3& C)
{   
  EV3 result;

  double a = sqrt(pow(B(0) - C(0), 2) + pow(B(1) - C(1), 2));
  double b = sqrt(pow(A(0) - C(0), 2) + pow(A(1) - C(1), 2));
  double c = sqrt(pow(A(0) - B(0), 2) + pow(A(1) - B(1), 2));

  result(0) = (a * A(0) + b * B(0) + c * C(0) ) / (a + b + c);
  result(1) = (a * A(1) + b * B(1) + c * C(1) ) / (a + b + c);
  result(2) = (A(2)+B(2)+C(2))/3.0;

  return result;
}
//ratio = BP/(BP+BA)
EV3 RobotBody::calLineSection(const EV3& A, const EV3& B, float ratio)
{
  EV3 result;
  result(0) = B(0) - ratio * (B(0) - A(0));
  result(1) = B(1) - ratio * (B(1) - A(1));

  return result;
}
/*
  line1:(x2-x1)(y-y1)-(y2-y1)(x-x1)=0 ;  line2:(x4-x3)(y-y3)-(y4-y3)(x-x3)=0
solution:
  x = [(y2-y1)(x4-x3)x1 - (y4-y3)(x2-x1)x3] / [(y2-y1)(x4-x3)-(y4-y3)(x2-x1)]
  y = [(x2-x1)(y4-y3)y1 - (x4-x3)(y2-y1)y3] / [(x2-x1)(y4-y3)-(x4-x3)(y2-y1)]
*/
EV3 RobotBody::calCrossPoint(const EV3& P1, const EV3& P2, const EV3& P3, const EV3& P4)
{
  EV3 result;

  result(0) = ((P2(1)-P1(1)) * (P4(0)-P3(0)) * P1(0) - (P4(1)-P3(1)) * (P2(0)-P1(0)) * P3(0)) 
            / ((P2(1)-P1(1)) * (P4(0)-P3(0)) - (P4(1)-P3(1)) * (P2(0)-P1(0)));

  result(1) = ((P2(0)-P1(0)) * (P4(1)-P3(1)) * P1(1) - (P4(0)-P3(0)) * (P2(1)-P1(1)) * P3(1)) 
            / ((P2(0)-P1(0)) * (P4(1)-P3(1)) - (P4(0)-P3(0)) * (P2(1)-P1(1)));

  return result;
}

bool RobotBody::calInnerTriangle(const EV3& A, const EV3& B, const EV3& C, EM3& Triangle)
{  
  EV3 innerheart, a, b, c;
  float radius = this->calInscribedCircleRadius(A,B,C);  
  bool calculable = (radius>cog_threshold_) ? true:false;
  float ratio = (radius-cog_threshold_)/radius;    

  if(calculable)
  {
    innerheart = this->calInnerHeart(A, B, C);
   
    a = this->calLineSection(A, innerheart, ratio);
    b = this->calLineSection(B, innerheart, ratio);   
    c = this->calLineSection(C, innerheart, ratio);  

    Triangle << a,b,c;
    Triangle.transpose();
  }
  return calculable;
}
/*  
Description: after design the exact trajectory, providing velocity while move the body
Formula：(might as well think t1=0 and t2=duration_ to reduce calculation)
  X-axis:
    Vel(t):6 * XD * t^5 / duration_^5 - 15 * XD * t^4 / duration_^4 + 10 * XD * t^3 / duration_^3
  Y-axis:
    Vel(t):6 * YD * t^5 / duration_^5 - 15 * YD * t^4 / duration_^4 + 10 * YD * t^3 / duration_^3
  Meantime,XD,YD is the distance to the desired position,duration_ is the stance time for CoG adjusting
*/
EV3 RobotBody::calCogPos(const EV3& dist, const int& t)
{  
  EV3 result(0,0,0);
  result(0) = 6  * dist(0) * pow(t,5) / pow(duration_,5) 
           - 15 * dist(0) * pow(t,4) / pow(duration_,4)
           + 10 * dist(0) * pow(t,3) / pow(duration_,3);
  result(1) = 6  * dist(1) * pow(t,5) / pow(duration_,5) 
           - 15 * dist(1) * pow(t,4) / pow(duration_,4)
           + 10 * dist(1) * pow(t,3) / pow(duration_,3);
  return result;
}

EV3 RobotBody::calCogVel(const EV3& dist, const int& t)
{ 
  EV3 result(0,0,0);
  result(0) = 30 * dist(0) * pow(t,4) / pow(duration_,5) 
           - 60 * dist(0) * pow(t,3) / pow(duration_,4)
           + 30 * dist(0) * pow(t,2) / pow(duration_,3);
  result(1) = 30 * dist(1) * pow(t,4) / pow(duration_,5) 
           - 60 * dist(1) * pow(t,3) / pow(duration_,4)
           + 30 * dist(1) * pow(t,2) / pow(duration_,3);
  return result;
}
} /* namespace qr_control */

/*#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::RobotBody, Label)*/

#endif
