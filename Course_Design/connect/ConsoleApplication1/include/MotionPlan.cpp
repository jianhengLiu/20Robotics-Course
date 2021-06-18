#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "../../eigen3/Eigen/Dense"

using namespace std;
using namespace HLRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}
	for (int i = 0; i < 3; i++)
	{
		mConfig[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

void CHLMotionPlan::SetProfileCartesian(double vel, double acc)
{
	tVel = vel;
	tAcc = acc;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和结束点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	double startAngle[3], endAngle[3];

	startAngle[0] = startPos.yaw * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll * PI / 180;

	endAngle[0] = endPos.yaw * PI / 180;
	endAngle[1] = endPos.pitch * PI / 180;
	endAngle[2] = endPos.roll * PI / 180;

	mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] = cos(endAngle[0]) * cos(endAngle[1]) * cos(endAngle[2]) - sin(endAngle[0]) * sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0]) * cos(endAngle[1]) * sin(endAngle[2]) - sin(endAngle[0]) * cos(endAngle[2]);
	mEndMatrixData[2] = cos(endAngle[0]) * sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] = sin(endAngle[0]) * cos(endAngle[1]) * cos(endAngle[2]) + cos(endAngle[0]) * sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0]) * cos(endAngle[1]) * sin(endAngle[2]) + cos(endAngle[0]) * cos(endAngle[2]);
	mEndMatrixData[6] = sin(endAngle[0]) * sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1]) * cos(endAngle[2]);
	mEndMatrixData[9] = sin(endAngle[1]) * sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;

	double angle[6] = { 0 };
	//HLRobot::SetRobotPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
	HLRobot::GetJointAngles(mStartMatrixData, angle);

	mJointAngleBegin[0] = angle[0];
	mJointAngleBegin[1] = angle[1];
	mJointAngleBegin[2] = angle[2];
	mJointAngleBegin[3] = angle[3];
	mJointAngleBegin[4] = angle[4];
	mJointAngleBegin[5] = angle[5];

	//HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll, endPos.config);
	HLRobot::GetJointAngles(mEndMatrixData, angle);
	mJointAngleEnd[0] = angle[0];
	mJointAngleEnd[1] = angle[1];
	mJointAngleEnd[2] = angle[2];
	mJointAngleEnd[3] = angle[3];
	mJointAngleEnd[4] = angle[4];
	mJointAngleEnd[5] = angle[5];

}

/********************************************************************
ABSTRACT:	运动轨迹规划部分

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints()
{
	ofstream outfile;               			//创建文件
	outfile.open("data.txt");
	outfile << mJointAngleBegin[0] << "  "
		<< mJointAngleBegin[1] << "  "
		<< mJointAngleBegin[2] << "  "
		<< mJointAngleBegin[3] << "  "
		<< mJointAngleBegin[4] << "  "
		<< mJointAngleBegin[5] << "  ";
	outfile << endl;                           //保存初始的时间、六个关节角度

	double joint_angle[6] = { 0 };
	double joint_time[6] = { 0 };
	int num_point[6] = { 0 };
	double joint_acc[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		joint_angle[i] = fabs(mJointAngleBegin[i] - mJointAngleEnd[i]);

		if (mJointAngleBegin[i] > mJointAngleEnd[i])
		{
			joint_acc[i] = -mAcc;
		}
		else
		{
			joint_acc[i] = mAcc;
		}

		//三角形
		if (joint_angle[i] <= mVel * mVel / mAcc)
		{
			joint_time[i] = 2 * sqrt(joint_angle[i] / mAcc);
		}
		//梯形
		else
		{
			joint_time[i] = 2 * mVel / mAcc + (joint_angle[i] / mVel - mVel / mAcc);
		}

		num_point[i] = joint_time[i] / mSampleTime;
	}

	int num_point_max = 0;
	for (int i = 0; i < 6; i++)
	{
		if (num_point[i] > num_point_max)
			num_point_max = num_point[i];
	}

	double joint_pos[6] = { 0 };  //每个轴当前关节角
	for (int i = 0; i < num_point_max; i++)
	{
		for (int j = 0; j < 6; j++)
		{

			//三角形规划
			if (joint_angle[j] <= mVel * mVel / mAcc)
			{
				if (i < 1000 * sqrt(joint_angle[j] / mAcc))
				{
					joint_pos[j] = mJointAngleBegin[j] + 0.5 * joint_acc[j] * i * i * 0.001 * 0.001;
				}
				else if (i >= 1000 * sqrt(joint_angle[j] / mAcc) && i < num_point[j])
				{
					joint_pos[j] = mJointAngleEnd[j] - 0.5 * joint_acc[j] * (num_point[j] - i) * (num_point[j] - i) * 0.001 * 0.001;
				}
				else
				{
					joint_pos[j] = joint_pos[j];
				}
			}
			//梯形规划
			else
			{
				if (i < 1000 * mVel / mAcc)
				{
					joint_pos[j] = mJointAngleBegin[j] + 0.5 * joint_acc[j] * i * i * 0.001 * 0.001;
				}
				else if (i >= 1000 * mVel / mAcc && i < num_point[j] - 1000 * mVel / mAcc)
				{
					joint_pos[j] = mJointAngleBegin[j] + joint_acc[j] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
				}
				else if (i >= num_point[j] - 1000 * mVel / mAcc && i < num_point[j])
				{
					joint_pos[j] = mJointAngleEnd[j] - 0.5 * joint_acc[j] * (num_point[j] - i) * (num_point[j] - i) * 0.001 * 0.001;
				}
				else
				{
					joint_pos[j] = joint_pos[j];
				}
			}
		}

		outfile << joint_pos[0] << "  "
			<< joint_pos[1] << "  "
			<< joint_pos[2] << "  "
			<< joint_pos[3] << "  "
			<< joint_pos[4] << "  "
			<< joint_pos[5] << "  ";
		outfile << endl;
	}
	outfile.close();
	cout << "Motion Planning in Joint Coordinate is finished!" << endl;
}


void CHLMotionPlan::GetPlanPointsCartesian(PosStruct startPos, PosStruct endPos)
{
	ofstream outfile;               			//创建文件
	outfile.open("data2.txt");
	outfile << mJointAngleBegin[0] << "  "
		<< mJointAngleBegin[1] << "  "
		<< mJointAngleBegin[2] << "  "
		<< mJointAngleBegin[3] << "  "
		<< mJointAngleBegin[4] << "  "
		<< mJointAngleBegin[5] << "  ";
	outfile << endl;                           //保存初始的时间、六个关节角度

	double tool_difference[6] = { 0 };    //每个笛卡尔空间维度的差
	double tool_acc[6] = { 0 };
	double tool_time[6] = { 0 };
	int num_point[6] = { 0 };

	tool_difference[0] = fabs(startPos.x - endPos.x);
	tool_difference[1] = fabs(startPos.y - endPos.y);
	tool_difference[2] = fabs(startPos.z - endPos.z);
	tool_difference[3] = fabs(startPos.yaw - endPos.yaw);
	tool_difference[4] = fabs(startPos.pitch - endPos.pitch);
	tool_difference[5] = fabs(startPos.roll - endPos.roll);

	if (startPos.x > endPos.x)
	{
		tool_acc[0] = -tAcc;
	}
	else
	{
		tool_acc[0] = tAcc;
	}

	if (startPos.y > endPos.y)
	{
		tool_acc[1] = -tAcc;
	}
	else
	{
		tool_acc[1] = tAcc;
	}

	if (startPos.z > endPos.z)
	{
		tool_acc[2] = -tAcc;
	}
	else
	{
		tool_acc[2] = tAcc;
	}

	if (startPos.yaw > endPos.yaw)
	{
		tool_acc[3] = -mAcc;
	}
	else
	{
		tool_acc[3] = mAcc;
	}
	if (startPos.pitch > endPos.pitch)
	{
		tool_acc[4] = -mAcc;
	}
	else
	{
		tool_acc[4] = mAcc;
	}
	if (startPos.roll > endPos.roll)
	{
		tool_acc[5] = -mAcc;
	}


	{
		//x,y,z
		for (int i = 0; i < 3; i++)
		{
			if (tool_difference[i] / 1000 < tVel * tVel / tAcc)
			{
				tool_time[i] = 2 * sqrt(tool_difference[i] * 0.001 / tAcc);
			}
			else
			{
				tool_time[i] = 2 * tVel / tAcc + (tool_difference[i] * 0.001 / tVel - tVel * tVel / tAcc);
			}
		}

		//yaw,pitch,roll
		for (int i = 3; i < 6; i++)
		{
			if (tool_difference[i] < mVel * mVel / mAcc)
			{
				tool_time[i] = 2 * sqrt(tool_difference[i] / mAcc);
			}
			else
			{
				tool_time[i] = 2 * mVel / mAcc + (tool_difference[i] - mVel * mVel / mAcc) / mVel;
			}
		}
	}


	for (int i = 0; i < 6; i++)
	{
		num_point[i] = tool_time[i] / mSampleTime;
	}

	int num_point_max = -1;
	for (int i = 0; i < 6; i++)
	{
		if (num_point[i] > num_point_max)
			num_point_max = num_point[i];
	}


	double tool_status[6];
	tool_status[0] = startPos.x;
	tool_status[1] = startPos.y;
	tool_status[2] = startPos.z;
	tool_status[3] = startPos.yaw;
	tool_status[4] = startPos.pitch;
	tool_status[5] = startPos.roll;


	for (int i = 0; i < num_point_max; i++)
	{
		//x			
		if (tool_difference[0] / 1000 <= tVel * tVel / tAcc)
		{
			if (i < 1000 * sqrt(tool_difference[0] * 0.001 / tAcc))
			{
				tool_status[0] = startPos.x / 1000 + 0.5 * tool_acc[0] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(tool_difference[0] * 0.001 / tAcc) && i < num_point[0])
			{
				tool_status[0] = endPos.x / 1000 - 0.5 * tool_acc[0] * (num_point[0] - i) * (num_point[0] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[0] = tool_status[0];
			}
		}
		else
		{
			if (i < 1000 * tVel / tAcc)
			{
				tool_status[0] = startPos.x / 1000 + 0.5 * tool_acc[0] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * tVel / tAcc && i < num_point[0] - 1000 * tVel / tAcc)
			{
				tool_status[0] = startPos.x / 1000 + tool_acc[0] * (tVel / tAcc) * (i * 0.001 - 0.5 * tVel / tAcc);
			}
			else if (i >= num_point[0] - 1000 * tVel / tAcc && i < num_point[0])
			{
				tool_status[0] = endPos.x / 1000 - 0.5 * tool_acc[0] * (num_point[0] - i) * (num_point[0] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[0] = tool_status[0];
			}
		}
		//y			
		if (tool_difference[1] / 1000 <= tVel * tVel / tAcc)
		{
			if (i < 1000 * sqrt(tool_difference[1] * 0.001 / tAcc))
			{
				tool_status[1] = startPos.y / 1000 + 0.5 * tool_acc[1] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(tool_difference[1] * 0.001 / tAcc) && i < num_point[1])
			{
				tool_status[1] = endPos.y / 1000 - 0.5 * tool_acc[1] * (num_point[1] - i) * (num_point[1] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[1] = tool_status[1];
			}
		}
		else
		{
			if (i < 1000 * tVel / tAcc)
			{
				tool_status[1] = startPos.y / 1000 + 0.5 * tool_acc[1] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * tVel / tAcc && i < num_point[1] - 1000 * tVel / tAcc)
			{
				tool_status[1] = startPos.y / 1000 + tool_acc[1] * (tVel / tAcc) * (i * 0.001 - 0.5 * tVel / tAcc);
			}
			else if (i >= num_point[1] - 1000 * tVel / tAcc && i < num_point[1])
			{
				tool_status[1] = endPos.y / 1000 - 0.5 * tool_acc[1] * (num_point[1] - i) * (num_point[1] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[1] = tool_status[1];
			}
		}
		//z
		if (tool_difference[2] / 1000 <= tVel * tVel / tAcc)
		{
			if (i < 1000 * sqrt(tool_difference[2] * 0.001 / tAcc))
			{
				tool_status[2] = startPos.z / 1000 + 0.5 * tool_acc[2] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(tool_difference[2] * 0.001 / tAcc) && i < num_point[2])
			{
				tool_status[2] = endPos.z / 1000 - 0.5 * tool_acc[2] * (num_point[2] - i) * (num_point[2] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[2] = tool_status[2];
			}
		}
		else
		{
			if (i < 1000 * tVel / tAcc)
			{
				tool_status[2] = startPos.z / 1000 + 0.5 * tool_acc[2] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * tVel / tAcc && i < num_point[2] - 1000 * tVel / tAcc)
			{
				tool_status[2] = startPos.z / 1000 + tool_acc[2] * (tVel / tAcc) * (i * 0.001 - 0.5 * tVel / tAcc);
			}
			else if (i >= num_point[2] - 1000 * tVel / tAcc && i < num_point[2])
			{
				tool_status[2] = endPos.z / 1000 - 0.5 * tool_acc[2] * (num_point[2] - i) * (num_point[2] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[2] = tool_status[2];
			}
		}
		//yaw
		if (tool_difference[3] <= mVel * mVel / mAcc)
		{
			if (i < 1000 * sqrt(tool_difference[3] / mAcc))
			{
				tool_status[3] = startPos.yaw + 0.5 * tool_acc[3] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(tool_difference[3] / mAcc) && i < num_point[3])
			{
				tool_status[3] = endPos.yaw - 0.5 * tool_acc[3] * (num_point[3] - i) * (num_point[3] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[3] = tool_status[3];
			}
		}
		else
		{
			if (i < 1000 * mVel / mAcc)
			{
				tool_status[3] = startPos.yaw + 0.5 * tool_acc[3] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * mVel / mAcc && i < num_point[3] - 1000 * mVel / mAcc)
			{
				tool_status[3] = startPos.yaw + tool_acc[3] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
			}
			else if (i >= num_point[3] - 1000 * mVel / mAcc && i < num_point[3])
			{
				tool_status[3] = endPos.yaw - 0.5 * tool_acc[3] * (num_point[3] - i) * (num_point[3] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[3] = tool_status[3];
			}

		}
		//pitch
		if (tool_difference[4] <= mVel * mVel / mAcc)
		{
			if (i < 1000 * sqrt(tool_difference[4] / mAcc))
			{
				tool_status[4] = startPos.pitch + 0.5 * tool_acc[4] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(tool_difference[4] / mAcc) && i < num_point[4])
			{
				tool_status[4] = endPos.pitch - 0.5 * tool_acc[4] * (num_point[4] - i) * (num_point[4] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[4] = tool_status[4];
			}
		}
		else
		{
			if (i < 1000 * mVel / mAcc)
			{
				tool_status[4] = startPos.pitch + 0.5 * tool_acc[4] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * mVel / mAcc && i < num_point[4] - 1000 * mVel / mAcc)
			{
				tool_status[4] = startPos.pitch + tool_acc[4] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
			}
			else if (i >= num_point[4] - 1000 * mVel / mAcc && i < num_point[4])
			{
				tool_status[4] = endPos.pitch - 0.5 * tool_acc[4] * (num_point[4] - i) * (num_point[4] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[4] = tool_status[4];
			}
		}
		//roll
		if (tool_difference[5] <= mVel * mVel / mAcc)
		{
			if (i < 1000 * sqrt(tool_difference[5] / mAcc))
			{
				tool_status[5] = startPos.roll + 0.5 * tool_acc[5] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(tool_difference[5] / mAcc) && i < num_point[5])
			{
				tool_status[5] = endPos.roll - 0.5 * tool_acc[5] * (num_point[5] - i) * (num_point[5] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[5] = tool_status[5];
			}
		}
		else
		{
			if (i < 1000 * mVel / mAcc)
			{
				tool_status[5] = startPos.roll + 0.5 * tool_acc[5] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * mVel / mAcc && i < num_point[5] - 1000 * mVel / mAcc)
			{
				tool_status[5] = startPos.roll + tool_acc[5] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
			}
			else if (i >= num_point[5] - 1000 * mVel / mAcc && i < num_point[5])
			{
				tool_status[5] = endPos.roll - 0.5 * tool_acc[5] * (num_point[5] - i) * (num_point[5] - i) * 0.001 * 0.001;
			}
			else
			{
				tool_status[5] = tool_status[5];
			}
		}

		double startAngle[3], endAngle[3];

		startAngle[0] = tool_status[3] * PI / 180;
		startAngle[1] = tool_status[4] * PI / 180;
		startAngle[2] = tool_status[5] * PI / 180;


		mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);
		mStartMatrixData[3] = tool_status[0];

		mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);
		mStartMatrixData[7] = tool_status[1];

		mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
		mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
		mStartMatrixData[10] = cos(startAngle[1]);
		mStartMatrixData[11] = tool_status[2];

		mStartMatrixData[12] = 0;
		mStartMatrixData[13] = 0;
		mStartMatrixData[14] = 0;
		mStartMatrixData[15] = 1;

		double joint_angle[6] = { 0 };
		HLRobot::GetJointAngles(mStartMatrixData, joint_angle);

		//输出到文件
		outfile << joint_angle[0] << "  "
			<< joint_angle[1] << "  "
			<< joint_angle[2] << "  "
			<< joint_angle[3] << "  "
			<< joint_angle[4] << "  "
			<< joint_angle[5] << "  ";
		outfile << endl;
	}
	cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;

}

void CHLMotionPlan::GetPlanPoints(int k)
{
    string s = "data.txt";
    ostringstream os;
    os << k << s;

    ofstream outfile;               			//创建文件
    outfile.open(os.str());
    outfile << mJointAngleBegin[0] << "  "
        << mJointAngleBegin[1] << "  "
        << mJointAngleBegin[2] << "  "
        << mJointAngleBegin[3] << "  "
        << mJointAngleBegin[4] << "  "
        << mJointAngleBegin[5] << "  ";
    outfile << endl;                           //保存初始的时间、六个关节角度

    double joint_angle[6] = { 0 };
    double joint_time[6] = { 0 };
    int num_point[6] = { 0 };
    double joint_acc[6] = { 0 };
    for (int i = 0; i < 6; i++)
    {
        joint_angle[i] = fabs(mJointAngleBegin[i] - mJointAngleEnd[i]);

        if (mJointAngleBegin[i] > mJointAngleEnd[i])
        {
            joint_acc[i] = -mAcc;
        }
        else
        {
            joint_acc[i] = mAcc;
        }

        //三角形
        if (joint_angle[i] <= mVel * mVel / mAcc)
        {
            joint_time[i] = 2 * sqrt(joint_angle[i] / mAcc);
        }
        //梯形
        else
        {
            joint_time[i] = 2 * mVel / mAcc + (joint_angle[i] / mVel - mVel / mAcc);
        }

        num_point[i] = joint_time[i] / mSampleTime;
    }

    int num_point_max = 0;
    for (int i = 0; i < 6; i++)
    {
        if (num_point[i] > num_point_max)
            num_point_max = num_point[i];
    }

    double joint_pos[6] = { 0 };  //每个轴当前关节角
    for (int i = 0; i < num_point_max; i++)
    {
        for (int j = 0; j < 6; j++)
        {

            //三角形规划
            if (joint_angle[j] <= mVel * mVel / mAcc)
            {
                if (i <= 1000 * sqrt(joint_angle[j] / mAcc))
                {
                    joint_pos[j] = mJointAngleBegin[j] + 0.5 * joint_acc[j] * i * i * 0.001 * 0.001;
                }
                else if (i > 1000 * sqrt(joint_angle[j] / mAcc) && i <= num_point[j])
                {
                    joint_pos[j] = mJointAngleEnd[j] - 0.5 * joint_acc[j] * (num_point[j] - i) * (num_point[j] - i) * 0.001 * 0.001;
                }
                else
                {
                    joint_pos[j] = joint_pos[j];
                }
            }
            //梯形规划
            else
            {
                if (i <= 1000 * mVel / mAcc)
                {
                    joint_pos[j] = mJointAngleBegin[j] + 0.5 * joint_acc[j] * i * i * 0.001 * 0.001;
                }
                else if (i > 1000 * mVel / mAcc && i <= num_point[j] - 1000 * mVel / mAcc)
                {
                    joint_pos[j] = mJointAngleBegin[j] + joint_acc[j] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
                }
                else if (i > num_point[j] - 1000 * mVel / mAcc && i <= num_point[j])
                {
                    joint_pos[j] = mJointAngleEnd[j] - 0.5 * joint_acc[j] * (num_point[j] - i) * (num_point[j] - i) * 0.001 * 0.001;
                }
                else
                {
                    joint_pos[j] = joint_pos[j];
                }
            }
        }

        outfile << joint_pos[0] << "  "
            << joint_pos[1] << "  "
            << joint_pos[2] << "  "
            << joint_pos[3] << "  "
            << joint_pos[4] << "  "
            << joint_pos[5] << "  ";
        outfile << endl;
    }
    outfile.close();
    cout << "Motion Planning in Joint Coordinate is finished!" << endl;
}

void CHLMotionPlan::GetPlanPointsCartesianLine(PosStruct startPos, PosStruct endPos,int i)
{
    string s= "data.txt";
    ostringstream os;
    os << i << s;

	ofstream outfile;               			//创建文件
	outfile.open(os.str());
	outfile << mJointAngleBegin[0] << "  "
		<< mJointAngleBegin[1] << "  "
		<< mJointAngleBegin[2] << "  "
		<< mJointAngleBegin[3] << "  "
		<< mJointAngleBegin[4] << "  "
		<< mJointAngleBegin[5] << "  ";
	outfile << endl;

	double tool_difference[3] = { 0 };
	double tool_direction[3] = { 0 };

	int num_point = 0;

	tool_difference[0] = (endPos.x - startPos.x) / 1000;
	tool_difference[1] = (endPos.y - startPos.y) / 1000;
	tool_difference[2] = (endPos.z - startPos.z) / 1000;

	double distance = sqrt(tool_difference[0] * tool_difference[0] + tool_difference[1] * tool_difference[1] + tool_difference[2] * tool_difference[2]);
	if (distance > 0)
	{
		double Time = 0;
		double delta = 0;

		tool_direction[0] = tool_difference[0] / distance;
		tool_direction[1] = tool_difference[1] / distance;
		tool_direction[2] = tool_difference[2] / distance;

		if (distance < tVel * tVel / tAcc)
		{
			Time = 2 * sqrt(tVel / tAcc);
		}
		else
		{
			Time = 2 * tVel / tAcc + (distance / tVel - tVel / tAcc);
		}

		num_point = Time / mSampleTime;

		for (int i = 0; i < num_point; i++)
		{
			if (distance <= tVel * tVel / tAcc)
			{
				if (i < 1000 * sqrt(distance / tAcc))
				{
					delta = 0.5 * tAcc * i * i * 0.001 * 0.001;
					mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
					mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
					mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
				}
				else if (i >= 1000 * sqrt(distance / tAcc) && i < num_point)
				{
					delta = distance - 0.5 * tAcc * i * i * 0.001 * 0.001;
					mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
					mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
					mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
				}
			}
			else
			{
				if (i < 1000 * tVel / tAcc)
				{
					delta = 0.5 * tAcc * i * i * 0.001 * 0.001;
					mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
					mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
					mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
				}
				else if (i >= 1000 * tVel / tAcc && i < num_point - 1000 * tVel / tAcc)
				{
					delta = tAcc * (tVel / tAcc) * (i * 0.001 - 0.5 * tVel / tAcc);
					mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
					mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
					mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
				}
				else if (i >= num_point - 1000 * tVel / tAcc && i < num_point)
				{
					delta = distance - 0.5 * tAcc * (num_point - i) * (num_point - i) * 0.001 * 0.001;
					mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
					mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
					mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
				}
			}


			double startAngle[3], endAngle[3];

			startAngle[0] = startPos.yaw * PI / 180;
			startAngle[1] = startPos.pitch * PI / 180;
			startAngle[2] = startPos.roll * PI / 180;


			mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
			mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
			mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);

			mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
			mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
			mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);

			mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
			mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
			mStartMatrixData[10] = cos(startAngle[1]);

			mStartMatrixData[12] = 0;
			mStartMatrixData[13] = 0;
			mStartMatrixData[14] = 0;
			mStartMatrixData[15] = 1;

			double angle[6] = { 0 };
			//HLRobot::SetRobotPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
			HLRobot::GetJointAngles(mStartMatrixData, angle);

			outfile << angle[0] << "  "
				<< angle[1] << "  "
				<< angle[2] << "  "
				<< angle[3] << "  "
				<< angle[4] << "  "
				<< angle[5] << "  ";
			outfile << endl;
		}

	}
	outfile.close();
}

void CHLMotionPlan::GetPlanPointsCartesian(PosStruct startPos, PosStruct endPos, ofstream& outfile)
{
	outfile << mJointAngleBegin[0] << "  "
		<< mJointAngleBegin[1] << "  "
		<< mJointAngleBegin[2] << "  "
		<< mJointAngleBegin[3] << "  "
		<< mJointAngleBegin[4] << "  "
		<< mJointAngleBegin[5] << "  ";
	outfile << endl;

	double tool_difference[3] = { 0 };
	double tool_direction[3] = { 0 };//

	int num_point = 0;

	tool_difference[0] = (endPos.x - startPos.x) / 1000;
	tool_difference[1] = (endPos.y - startPos.y) / 1000;
	tool_difference[2] = (endPos.z - startPos.z) / 1000;

	double distance = sqrt(tool_difference[0] * tool_difference[0] + tool_difference[1] * tool_difference[1] + tool_difference[2] * tool_difference[2]);
	double Time = 0;
	double delta = 0;

	tool_direction[0] = tool_difference[0] / distance;
	tool_direction[1] = tool_difference[1] / distance;
	tool_direction[2] = tool_difference[2] / distance;

	if (distance < tVel * tVel / tAcc)
	{
		Time = 2 * sqrt(tVel / tAcc);
	}
	else
	{
		Time = 2 * tVel / tAcc + (distance / tVel - tVel / tAcc);
	}

	num_point = Time / mSampleTime;

	for (int i = 0; i < num_point; i++)
	{
		if (distance <= tVel * tVel / tAcc)
		{
			if (i < 1000 * sqrt(distance / tAcc))
			{
				delta = 0.5 * tAcc * i * i * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
			}
			else if (i >= 1000 * sqrt(distance / tAcc) && i < num_point)
			{
				delta = distance - 0.5 * tAcc * i * i * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
			}
		}
		else
		{
			if (i < 1000 * tVel / tAcc)
			{
				delta = 0.5 * tAcc * i * i * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
			}
			else if (i >= 1000 * tVel / tAcc && i < num_point - 1000 * tVel / tAcc)
			{
				delta = tAcc * (tVel / tAcc) * (i * 0.001 - 0.5 * tVel / tAcc);
				mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
			}
			else if (i >= num_point - 1000 * tVel / tAcc && i < num_point)
			{
				delta = distance - 0.5 * tAcc * (num_point - i) * (num_point - i) * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * tool_direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * tool_direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * tool_direction[2];
			}
		}


		double startAngle[3], endAngle[3];

		startAngle[0] = startPos.yaw * PI / 180;
		startAngle[1] = startPos.pitch * PI / 180;
		startAngle[2] = startPos.roll * PI / 180;


		mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);

		mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);

		mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
		mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
		mStartMatrixData[10] = cos(startAngle[1]);

		mStartMatrixData[12] = 0;
		mStartMatrixData[13] = 0;
		mStartMatrixData[14] = 0;
		mStartMatrixData[15] = 1;

		double angle[6] = { 0 };
		//HLRobot::SetRobotPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
		HLRobot::GetJointAngles(mStartMatrixData, angle);

		outfile << angle[0] << "  "
			<< angle[1] << "  "
			<< angle[2] << "  "
			<< angle[3] << "  "
			<< angle[4] << "  "
			<< angle[5] << "  ";
		outfile << endl;
	}
}


void CHLMotionPlan::GetPlanPoints(ofstream& outfile)
{
	outfile << mJointAngleBegin[0] << "  "
		<< mJointAngleBegin[1] << "  "
		<< mJointAngleBegin[2] << "  "
		<< mJointAngleBegin[3] << "  "
		<< mJointAngleBegin[4] << "  "
		<< mJointAngleBegin[5] << "  ";
	outfile << endl;                           //保存初始的时间、六个关节角度

	double joint_angle[6] = { 0 };
	double joint_time[6] = { 0 };
	int num_point[6] = { 0 };
	double joint_acc[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		joint_angle[i] = fabs(mJointAngleBegin[i] - mJointAngleEnd[i]);

		if (mJointAngleBegin[i] > mJointAngleEnd[i])
		{
			joint_acc[i] = -mAcc;
		}
		else
		{
			joint_acc[i] = mAcc;
		}

		//三角形
		if (joint_angle[i] <= mVel * mVel / mAcc)
		{
			joint_time[i] = 2 * sqrt(joint_angle[i] / mAcc);
		}
		//梯形
		else
		{
			joint_time[i] = 2 * mVel / mAcc + (joint_angle[i] / mVel - mVel / mAcc);
		}

		num_point[i] = joint_time[i] / mSampleTime;
	}

	int num_point_max = 0;
	for (int i = 0; i < 6; i++)
	{
		if (num_point[i] > num_point_max)
			num_point_max = num_point[i];
	}

	double joint_pos[6] = { 0 };  //每个轴当前关节角
	for (int i = 0; i < num_point_max; i++)
	{
		for (int j = 0; j < 6; j++)
		{

			//三角形规划
			if (joint_angle[j] <= mVel * mVel / mAcc)
			{
				if (i < 1000 * sqrt(joint_angle[j] / mAcc))
				{
					joint_pos[j] = mJointAngleBegin[j] + 0.5 * joint_acc[j] * i * i * 0.001 * 0.001;
				}
				else if (i >= 1000 * sqrt(joint_angle[j] / mAcc) && i < num_point[j])
				{
					joint_pos[j] = mJointAngleEnd[j] - 0.5 * joint_acc[j] * (num_point[j] - i) * (num_point[j] - i) * 0.001 * 0.001;
				}
				else
				{
					joint_pos[j] = joint_pos[j];
				}
			}
			//梯形规划
			else
			{
				if (i < 1000 * mVel / mAcc)
				{
					joint_pos[j] = mJointAngleBegin[j] + 0.5 * joint_acc[j] * i * i * 0.001 * 0.001;
				}
				else if (i >= 1000 * mVel / mAcc && i < num_point[j] - 1000 * mVel / mAcc)
				{
					joint_pos[j] = mJointAngleBegin[j] + joint_acc[j] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
				}
				else if (i >= num_point[j] - 1000 * mVel / mAcc && i < num_point[j])
				{
					joint_pos[j] = mJointAngleEnd[j] - 0.5 * joint_acc[j] * (num_point[j] - i) * (num_point[j] - i) * 0.001 * 0.001;
				}
				else
				{
					joint_pos[j] = joint_pos[j];
				}
			}
		}

		outfile << joint_pos[0] << "  "
			<< joint_pos[1] << "  "
			<< joint_pos[2] << "  "
			<< joint_pos[3] << "  "
			<< joint_pos[4] << "  "
			<< joint_pos[5] << "  ";
		outfile << endl;
	}
}


void CHLMotionPlan::GetPlanPointsRect(PosStruct rect1, PosStruct rect2, PosStruct rect3, PosStruct rect4)
{
	ofstream outfile;               			//创建文件
	outfile.open("data.txt");

	SetPlanPoints(rect1, rect2);
	GetPlanPoints(outfile);
	SetPlanPoints(rect2, rect3);
	GetPlanPoints(outfile);
	SetPlanPoints(rect3, rect4);
	GetPlanPoints(outfile);
	SetPlanPoints(rect4, rect1);
	GetPlanPoints(outfile);

	outfile.close();
	cout << "Rectangle Motion Planning in Joint Coordinate is finished!" << endl;
}



void CHLMotionPlan::GetPlanPointsRectCartesianLine(PosStruct rect1, PosStruct rect2, PosStruct rect3, PosStruct rect4)
{
	{
		ofstream outfile;               			//创建文件
		outfile.open("data.txt");

		SetPlanPoints(rect1, rect2);
		GetPlanPoints(outfile);
		SetPlanPoints(rect2, rect3);
		GetPlanPoints(outfile);
		SetPlanPoints(rect3, rect4);
		GetPlanPoints(outfile);
		SetPlanPoints(rect4, rect1);
		GetPlanPoints(outfile);

		outfile.close();
		cout << "Rectangle Motion Planning in Joint Coordinate is finished!" << endl;
	}

}

