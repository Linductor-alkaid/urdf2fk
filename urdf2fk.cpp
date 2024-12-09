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
    cout << "X: " << T[1][3] << ", Y: " << T[2][3] << ", Z: " << T[0][3] << endl;
}

int main() {
    // 解析URDF文件
    string urdfFilename = "robot_model.urdf";  // URDF文件名

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
    vector<JointLinkParam> joints;
    vector<double> jointAlphas;

    // 解析每个 <joint> 元素，获取 alpha 值
    XMLElement* jointElement = robotElement->FirstChildElement("joint");

    while (jointElement) {
        JointLinkParam joint;
        joint.name = jointElement->Attribute("name");

        // 解析axis元素
        XMLElement* axisElement = jointElement->FirstChildElement("axis");
        if (axisElement) {
            const char* axis = axisElement->Attribute("xyz");

            // 解析axis
            if (axis) {
                double ax, ay, az;
                sscanf(axis, "%lf %lf %lf", &ax, &ay, &az);

                // 通过反三角函数计算alpha (arctan(y/x))
                if (ax == 0.0 && ay == 0.0) {
                    joint.alpha = 0.0;  // 如果轴是 (0, 0, z)，则alpha为0
                } else {
                    joint.alpha = atan2(ay, ax);  // 计算旋转角度 alpha
                }

                // 将计算出来的 alpha 存入 jointAlphas
                jointAlphas.push_back(joint.alpha);
            }
        }
        joints.push_back(joint);

        jointElement = jointElement->NextSiblingElement("joint");
    }

    // 存储所有的 Link 参数
    vector<JointLinkParam> links;
    XMLElement* linkElement = robotElement->FirstChildElement("link");

    while (linkElement) {
        JointLinkParam link;
        link.name = linkElement->Attribute("name");

        // 解析link中的inertial元素
        XMLElement* inertialElement = linkElement->FirstChildElement("inertial");
        if (inertialElement) {
            XMLElement* originElement = inertialElement->FirstChildElement("origin");
            if (originElement) {
                const char* xyz = originElement->Attribute("xyz");

                if (xyz) {
                    double x, y, z;
                    sscanf(xyz, "%lf %lf %lf", &x, &y, &z);
                    link.d = 2*x;  // 将 x 赋值给连杆偏移 d
                    link.a = 2*z;  // 将 z 赋值给连杆长度 a
                }
            }
        }

        // 根据 joint 中的 alpha 值设置
        if (!jointAlphas.empty()) {
            link.alpha = jointAlphas.front();
            jointAlphas.erase(jointAlphas.begin());
        } else{
            link.alpha = 0.0;
        }

        // 将解析出的 Link 信息存入 links
        links.push_back(link);

        linkElement = linkElement->NextSiblingElement("link");
    }

    // // 输出解析结果
    // cout << "Parsed Joint and Link Parameters:" << endl;
    // for (size_t i = 0; i < links.size(); ++i) {
    //     cout << "Link " << i << " (" << links[i].name << "):" << endl;
    //     cout << "  d (offset): " << links[i].d << endl;
    //     cout << "  a (length): " << links[i].a << endl;
    //     cout << "  alpha (twist): " << links[i].alpha << endl;
    // }

    // 用户输入关节角度
    vector<double> jointAngles;
    jointAngles.push_back(0.0);  // 将第一个关节的theta设为0
    for (size_t i = 0; i < joints.size(); ++i) {
        double angle;
        cout << "Enter the angle for joint " << joints[i].name << " (in radians): ";
        cin >> angle;
        jointAngles.push_back(angle);
    }

    // 计算末端执行器的位姿
    calculateEndEffectorFK(links, jointAngles);

    return 0;
}
