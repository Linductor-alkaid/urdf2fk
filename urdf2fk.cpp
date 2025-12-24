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
    // joint origin的旋转和平移
    double origin_rpy[3];  // joint origin的rpy旋转 (roll, pitch, yaw)
    double origin_xyz[3];  // joint origin的xyz平移
    bool has_origin_rpy;   // 是否有origin rpy
    bool has_origin_xyz;   // 是否有origin xyz
};

// 将RPY (Roll-Pitch-Yaw) 转换为旋转矩阵
// RPY顺序：先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll
void rpyToRotationMatrix(double roll, double pitch, double yaw, double R[3][3]) {
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    // 旋转矩阵：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

// 创建齐次变换矩阵（从旋转矩阵和平移向量）
void createHomogeneousMatrix(const double R[3][3], const double t[3], double H[4][4]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H[i][j] = R[i][j];
        }
        H[i][3] = t[i];
    }
    H[3][0] = 0;
    H[3][1] = 0;
    H[3][2] = 0;
    H[3][3] = 1;
}

// 矩阵乘法：result = A * B
void matrixMultiply(const double A[4][4], const double B[4][4], double result[4][4]) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            result[row][col] = 0;
            for (int k = 0; k < 4; ++k) {
                result[row][col] += A[row][k] * B[k][col];
            }
        }
    }
}

// 旋转矩阵，基于DH参数
void getTransformationMatrix(const JointLinkParam &param, double matrix[4][4]) {
    double cosTheta = cos(param.theta);
    double sinTheta = sin(param.theta);
    double cosAlpha = cos(param.alpha);
    double sinAlpha = sin(param.alpha);

    // DH变换矩阵
    double dhMatrix[4][4] = {
        {cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, param.a * cosTheta},
        {sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, param.a * sinTheta},
        {0, sinAlpha, cosAlpha, param.d},
        {0, 0, 0, 1}
    };

    // 如果有joint origin的旋转或平移，需要先应用
    if (param.has_origin_rpy || param.has_origin_xyz) {
        // 创建joint origin的变换矩阵
        double originR[3][3];
        double originT[3] = {0, 0, 0};
        
        if (param.has_origin_rpy) {
            rpyToRotationMatrix(param.origin_rpy[0], param.origin_rpy[1], param.origin_rpy[2], originR);
        } else {
            // 如果没有rpy，使用单位旋转矩阵
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    originR[i][j] = (i == j) ? 1.0 : 0.0;
                }
            }
        }
        
        if (param.has_origin_xyz) {
            originT[0] = param.origin_xyz[0];
            originT[1] = param.origin_xyz[1];
            originT[2] = param.origin_xyz[2];
        }
        
        double originMatrix[4][4];
        createHomogeneousMatrix(originR, originT, originMatrix);
        
        // 组合变换：先应用joint origin变换，再应用DH变换
        matrixMultiply(originMatrix, dhMatrix, matrix);
    } else {
        // 如果没有joint origin变换，直接使用DH矩阵
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix[i][j] = dhMatrix[i][j];
            }
        }
    }
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
        double newT[4][4];
        matrixMultiply(T, jointT, newT);

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
        
        // 初始化origin相关字段
        joint.has_origin_rpy = false;
        joint.has_origin_xyz = false;
        for (int i = 0; i < 3; ++i) {
            joint.origin_rpy[i] = 0.0;
            joint.origin_xyz[i] = 0.0;
        }

        // 解析joint的origin元素（包含rpy和xyz）
        XMLElement* originElement = jointElement->FirstChildElement("origin");
        if (originElement) {
            // 解析rpy (roll, pitch, yaw)
            const char* rpy = originElement->Attribute("rpy");
            if (rpy) {
                sscanf(rpy, "%lf %lf %lf", &joint.origin_rpy[0], &joint.origin_rpy[1], &joint.origin_rpy[2]);
                joint.has_origin_rpy = true;
            }
            
            // 解析xyz平移
            const char* xyz = originElement->Attribute("xyz");
            if (xyz) {
                sscanf(xyz, "%lf %lf %lf", &joint.origin_xyz[0], &joint.origin_xyz[1], &joint.origin_xyz[2]);
                joint.has_origin_xyz = true;
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
    // 注意：需要将joint信息与link信息关联起来
    // 在URDF中，joint连接parent link和child link
    // joint的origin定义了从parent link到child link的变换
    // 第一个link（base_link）没有对应的joint，从第二个link开始对应前一个joint
    vector<JointLinkParam> links;
    XMLElement* linkElement = robotElement->FirstChildElement("link");
    size_t linkIndex = 0;

    while (linkElement) {
        JointLinkParam link;
        link.name = linkElement->Attribute("name");
        
        // 初始化origin相关字段
        link.has_origin_rpy = false;
        link.has_origin_xyz = false;
        for (int i = 0; i < 3; ++i) {
            link.origin_rpy[i] = 0.0;
            link.origin_xyz[i] = 0.0;
        }

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
        
        // 将对应joint的origin信息复制到link中
        // 注意：第一个link（base_link，linkIndex=0）没有对应的joint
        // 从第二个link开始（linkIndex>=1），每个link对应jointIndex=linkIndex-1的joint
        if (linkIndex > 0 && (linkIndex - 1) < joints.size()) {
            size_t jointIndex = linkIndex - 1;
            link.has_origin_rpy = joints[jointIndex].has_origin_rpy;
            link.has_origin_xyz = joints[jointIndex].has_origin_xyz;
            for (int i = 0; i < 3; ++i) {
                link.origin_rpy[i] = joints[jointIndex].origin_rpy[i];
                link.origin_xyz[i] = joints[jointIndex].origin_xyz[i];
            }
        }

        // 将解析出的 Link 信息存入 links
        links.push_back(link);
        linkIndex++;

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
