#pragma once
#ifndef MOTIONPLAN_H_
#define MOTIONPLAN_H_
#include <vector>
using namespace std;

struct PosStruct
{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
	bool config[3]{ 1,1,1 };	// config, 表示机器人姿态
};

class CHLMotionPlan
{
private:
	double mJointAngleBegin[6];					//起始点位的关节角度,单位度
	double mJointAngleEnd[6];					//结束点位的关节角度，单位度
	double mStartMatrixData[16];				//起始点位的转换矩阵数组
	double mEndMatrixData[16];					//结束点位的转换矩阵数组
	double mSampleTime;							//采样点位，单位S
	double mVel;								//角速度，单位°/s
	double mAcc;								//加速度，单位°/s/s
	double mDec;								//减速度，单位°/ s / s
	double tVel;                                //线速度 ，单位m/s
	double tAcc;                                //线加速度，单位m/s/s
	bool   mConfig[3];							//机器人姿态


public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();

	void SetSampleTime(double sampleTime);		                    //设置采样时间
	void SetPlanPoints(PosStruct startPos, PosStruct endPos);		//输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double vel, double acc, double dec);			//设置运动参数，速度、加速度和减速度
	void SetProfileCartesian(double vel, double acc);                               //设置线速度，加速度
	void GetPlanPoints();
    void GetPlanPoints(int k);//获取轨迹规划后离散点位
	void GetPlanPointsCartesian(PosStruct startPos, PosStruct endPos); //获取轨迹规划后离散点位(笛卡尔空间）
	void GetPlanPointsCartesianLine(PosStruct startPos, PosStruct endPos,int i); //获取轨迹规划后离散点位(笛卡尔空间）(没有y,p,r跟踪的)
	void GetPlanPoints(ofstream& outfile);
	void GetPlanPointsRect(PosStruct rect1, PosStruct rect2, PosStruct rect3, PosStruct rect4);
	void GetPlanPointsCartesian(PosStruct startPos, PosStruct endPos, ofstream& outfile);
	void GetPlanPointsRectCartesianLine(PosStruct rect1, PosStruct rect2, PosStruct rect3, PosStruct rect4);
};

#endif