#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <tinyxml2.h>

using namespace std;
using namespace tinyxml2;

// 关节和连杆参数结构体
struct JointLinkParam {
    string name;   // 关节名称
    double theta;  // 关节角度
    double d;      // 连杆偏移
    double a;      // 连杆长度
    double alpha;  // 连杆扭转角度
    string axis;   // 关节旋转轴 (例如 "1 0 0")
};

// 旋转矩阵，基于DH参数
void getTransformationMatrix(const JointLinkParam &param, double matrix[4][4]) {
    double cosTheta = cos(param.theta);
    double sinTheta = sin(param.theta);
    double cosAlpha = cos(param.alpha);
    double sinAlpha = sin(param.alpha);

    matrix[0][0] = cosTheta;
    matrix[0][1] = -sinTheta * cosAlpha;
    matrix[0][2] = sinTheta * sinAlpha;
    matrix[0][3] = param.a * cosTheta;

    matrix[1][0] = sinTheta;
    matrix[1][1] = cosTheta * cosAlpha;
    matrix[1][2] = -cosTheta * sinAlpha;
    matrix[1][3] = param.a * sinTheta;

    matrix[2][0] = 0;
    matrix[2][1] = sinAlpha;
    matrix[2][2] = cosAlpha;
    matrix[2][3] = param.d;

    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
}

// 计算末端执行器的位姿（位置和姿态）
void calculateEndEffectorFK(const vector<JointLinkParam>& joints, vector<double> jointAngles) {
    // 初始的变换矩阵为单位矩阵
    double T[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    // 遍历所有关节，计算每个关节到末端执行器的变换矩阵
    for (size_t i = 0; i < joints.size(); ++i) {
        JointLinkParam joint = joints[i];
        joint.theta = jointAngles[i];  // 更新关节角度

        double jointT[4][4];
        getTransformationMatrix(joint, jointT);

        // 累乘变换矩阵
        double newT[4][4] = {{0}};
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                newT[row][col] = 0;
                for (int k = 0; k < 4; ++k) {
                    newT[row][col] += T[row][k] * jointT[k][col];
                }
            }
        }

        // 更新当前的变换矩阵
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                T[row][col] = newT[row][col];
            }
        }
    }

    // 打印末端执行器的三维坐标
    cout << "End-Effector Position: ";
    cout << "X: " << T[0][3] << ", Y: " << T[1][3] << ", Z: " << T[2][3] << endl;
}

int main() {
    // 解析URDF文件
    string urdfFilename = "robot_model.urdf";  // URDF文件名
    vector<JointLinkParam> joints;

    XMLDocument doc;
    if (doc.LoadFile(urdfFilename.c_str()) != XML_SUCCESS) {
        cerr << "Failed to load URDF file." << endl;
        return 1;
    }

    XMLElement* robotElement = doc.FirstChildElement("robot");
    if (!robotElement) {
        cerr << "Invalid URDF structure." << endl;
        return 1;
    }

    // 解析所有的关节
    XMLElement* jointElement = robotElement->FirstChildElement("joint");
    while (jointElement) {
        JointLinkParam joint;
        joint.name = jointElement->Attribute("name");

        // 解析origin元素
        XMLElement* originElement = jointElement->FirstChildElement("origin");
        if (originElement) {
            const char* xyz = originElement->Attribute("xyz");

            // 解析xyz
            if (xyz) {
                double x, y, z;
                sscanf(xyz, "%lf %lf %lf", &x, &y, &z);
                joint.d = x;  // 连杆偏移为 xyz 中的 x
                joint.a = z;  // 连杆长度为 xyz 中的 z
            }
        }

        // 解析axis元素
        XMLElement* axisElement = jointElement->FirstChildElement("axis");
        if (axisElement) {
            const char* axis = axisElement->Attribute("xyz");

            // 解析axis
            if (axis) {
                double ax, ay, az;
                sscanf(axis, "%lf %lf %lf", &ax, &ay, &az);

                // 根据旋转轴反计算alpha
                if (ax == 1.0 && ay == 0.0 && az == 0.0) {
                    joint.alpha = 0.0;  // 如果轴是 (1, 0, 0)，则alpha为0
                } else {
                    joint.alpha = atan2(ay, ax);  // 通过反三角函数计算alpha
                }
            }
        }

        // 将解析出的关节信息存入joints
        joints.push_back(joint);
        jointElement = jointElement->NextSiblingElement("joint");
    }

    // 用户输入关节角度
    vector<double> jointAngles;
    for (size_t i = 0; i < joints.size(); ++i) {
        double angle;
        cout << "Enter the angle for joint " << joints[i].name << " (in radians): ";
        cin >> angle;
        jointAngles.push_back(angle);
    }

    // 计算末端执行器的位姿
    calculateEndEffectorFK(joints, jointAngles);

    return 0;
}
